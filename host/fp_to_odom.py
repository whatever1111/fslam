#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# fp_to_odom.py — host-side localization interface bridge for the Deep
# Robotics M20. Runs on the dog host (ROS2 Foxy + drdds) in BOTH fslam mode
# and fixposition-only mode, and publishes the OEM-style localization topics.
#
# Priority & stamping policy (/ODOM leads, the others follow):
#
#   /ODOM             nav_msgs/Odometry — HIGHEST priority.
#       fixposition-only mode (relay_enable:=true, the default): every
#       /fixposition/odometry_enu message is relayed IMMEDIATELY with its own
#       (newest) stamp — never delayed, never blocked by cloud processing.
#       fslam mode (relay_enable:=false): the container's fusion node
#       publishes /ODOM DIRECTLY (no relay hop); this node then only
#       SUBSCRIBES to /ODOM as the pose/stamp source for the topics below.
#   /LOC_BODY_POINTS  sensor_msgs/PointCloud2
#       Each /LIDAR/POINTS scan is deskewed into base_link AT THE NEWEST
#       /ODOM STAMP: /IMU gyro integration for rotation + odometry body
#       velocity for translation, per point timestamp. Its header stamp
#       therefore equals the stamp of an actual /ODOM message (共享时间戳).
#       Runs in a background worker thread (latest-scan-wins) so the /ODOM
#       relay is never blocked; numpy releases the GIL during the math.
#       lidar_link ≡ base_link on the M20 (URDF static TFs identity), so no
#       extrinsic is applied by default.
#   /LOCATION_STATUS  drdds/msg/LocationStatus
#       Published right after each cloud with the SAME stamp; a wall-clock
#       fallback timer keeps it flowing while the lidar is silent (it must
#       keep reporting exactly that failure).
#
# NOTE: this file is the Python prototype; it will be ported to C++ later.
# The numeric helpers are module-level pure functions (no rclpy) so they
# translate 1:1 and can be unit-tested without ROS.

import math
import statistics
import threading
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

try:
    import numpy as np
except ImportError:  # pragma: no cover — deskew degrades to a frame relay
    np = None

# drdds is only installed on the dog host. Without it the node still runs,
# it just skips /LOCATION_STATUS and the leg-odom monitor.
try:
    from drdds.msg import LocationStatus, MotionInfo

    HAVE_DRDDS = True
except ImportError:  # pragma: no cover
    HAVE_DRDDS = False


# =============================================================================
# drdds LocationStatus semantics
#
# total_status (定位状态码) — per Deep Robotics spec:
#   0 未初始化 uninitialized — needs localization-init command / drmap apply
#   1 正常     normal
#   2 低质量   low quality   — degraded matching / inputs, keep watching
#   3 丢失     lost          — stop navigation, re-initialize localization
#
# Sub-field flags — PENDING OEM SPEC (待官方定义确认), current assumption:
#   exec_val / loss_val error flags : 0 = normal, 1 = error
#   input_val flags                 : 1 = input present/healthy, 0 = missing
# Adjust ONLY these constants when the spec for the sub-fields arrives.
# =============================================================================
TOTAL_UNINITIALIZED = 0
TOTAL_NORMAL = 1
TOTAL_LOW_QUALITY = 2
TOTAL_LOST = 3
ERR_OK = 0
ERR_ERROR = 1
INPUT_OK = 1
INPUT_MISSING = 0

# PointField.datatype → numpy dtype string
_PF_DATATYPE_TO_NP = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
# candidate names for the per-point time field, in preference order
_TIME_FIELD_CANDIDATES = ('timestamp', 'time', 't', 'time_stamp', 'stamp')


def build_cloud_dtype(fields, point_step):
    """Build a numpy structured dtype mirroring a PointCloud2 layout.

    Returns (dtype, time_field_name_or_None) or (None, None) if xyz missing.
    """
    names, formats, offsets = [], [], []
    for f in fields:
        base = _PF_DATATYPE_TO_NP.get(f.datatype)
        if base is None:
            return None, None
        names.append(f.name)
        formats.append(base if f.count == 1 else (base, (f.count,)))
        offsets.append(f.offset)
    if not {'x', 'y', 'z'}.issubset(names):
        return None, None
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': point_step})
    time_field = next((c for c in _TIME_FIELD_CANDIDATES if c in names), None)
    return dtype, time_field


def relative_point_times(t_points, t_scan):
    """Normalize per-point times to seconds relative to the scan stamp.

    Handles the two layouts seen in the wild: absolute epoch seconds
    (M20 lidar: float64 ≈ 1.78e9) and scan-relative seconds (< a few s).
    Returns None if the values look like neither (deskew is then skipped).
    """
    t = t_points.astype(np.float64)
    if not np.isfinite(t).all():
        return None
    med = float(np.median(t))
    if abs(med - t_scan) < 5.0:  # absolute epoch stamps
        return t - t_scan
    if 0.0 <= med < 5.0:  # already relative to scan start
        return t
    return None


def integrate_gyro_rotvec(imu_t, imu_w, t_ref, t_query):
    """Rotation of the body from t_ref to each t_query, as rotation vectors.

    Trapezoidal integration of body angular rate over the IMU samples,
    first-order (commutative) accumulation — exact enough for the ≤0.1 s
    scan window of a walking robot. Outside the sample range np.interp
    clamps (constant extrapolation).
    imu_t: (N,) sorted seconds; imu_w: (N,3) rad/s. Returns (M,3).
    """
    if imu_t.shape[0] < 2:
        w = imu_w[-1] if imu_t.shape[0] == 1 else np.zeros(3)
        return (t_query - t_ref)[:, None] * w[None, :]
    dt = np.diff(imu_t)
    cum = np.zeros_like(imu_w)
    cum[1:] = np.cumsum(0.5 * (imu_w[1:] + imu_w[:-1]) * dt[:, None], axis=0)
    theta_q = np.stack([np.interp(t_query, imu_t, cum[:, k]) for k in range(3)], axis=1)
    theta_ref = np.array([np.interp(t_ref, imu_t, cum[:, k]) for k in range(3)])
    return theta_q - theta_ref


def rotvec_to_matrices(rotvec):
    """Vectorized Rodrigues: rotation vectors (U,3) → matrices (U,3,3)."""
    rv = rotvec.astype(np.float64)
    angle = np.linalg.norm(rv, axis=1)
    safe = np.where(angle < 1e-12, 1.0, angle)
    ax = rv / safe[:, None]
    x, y, z = ax[:, 0], ax[:, 1], ax[:, 2]
    c, s = np.cos(angle), np.sin(angle)
    one_c = 1.0 - c
    mats = np.empty((rv.shape[0], 3, 3))
    mats[:, 0, 0] = c + x * x * one_c
    mats[:, 0, 1] = x * y * one_c - z * s
    mats[:, 0, 2] = x * z * one_c + y * s
    mats[:, 1, 0] = y * x * one_c + z * s
    mats[:, 1, 1] = c + y * y * one_c
    mats[:, 1, 2] = y * z * one_c - x * s
    mats[:, 2, 0] = z * x * one_c - y * s
    mats[:, 2, 1] = z * y * one_c + x * s
    mats[:, 2, 2] = c + z * z * one_c
    return mats


def deskew_points(xyz, rel_t, imu_t, imu_w, t_ref, vel_body):
    """Deskew: express every point in the base_link pose at t_ref.

    p_ref = R(ref→i) · p_i + v_body · (t_i − t_ref)

    Optimized for mechanical lidars where points come in columns sharing a
    timestamp: rotations are computed once per UNIQUE timestamp (~N/32 trig
    ops instead of N) and gathered; the heavy per-point pass is a single
    einsum in float32.
    xyz: (M,3); rel_t: (M,) seconds relative to t_ref;
    imu_t/imu_w: gyro history; vel_body: (3,) m/s or None. Returns (M,3) f32.
    """
    uniq, inv = np.unique(rel_t, return_inverse=True)
    rotvec = integrate_gyro_rotvec(imu_t, imu_w, t_ref, t_ref + uniq)
    mats = rotvec_to_matrices(rotvec).astype(np.float32)
    pts = xyz.astype(np.float32, copy=False)
    out = np.einsum('mij,mj->mi', mats[inv], pts)
    if vel_body is not None:
        out += (rel_t[:, None] * vel_body[None, :]).astype(np.float32)
    return out


class FixpositionOdomRelay(Node):
    def __init__(self):
        super().__init__("fixposition_odom_to_odom")

        # ---- /ODOM relay ----
        # relay_enable=true : fixposition-only mode — relay input_topic → output_topic.
        # relay_enable=false: fslam mode — /ODOM comes directly from the container's
        #                     fusion node; only subscribe output_topic as the source.
        self.declare_parameter("relay_enable", True)
        self.declare_parameter("input_topic", "/fixposition/odometry_enu")
        self.declare_parameter("output_topic", "/ODOM")
        self.declare_parameter("input_depth", 10)
        self.declare_parameter("output_depth", 10)
        # OEM /ODOM carries an EMPTY child_frame_id (verified in dog bags)
        self.declare_parameter("target_child_frame_id", "")
        # RTK liveness source for input_val.rtk (independent of the relay wiring)
        self.declare_parameter("rtk_topic", "/fixposition/odometry_enu")
        # ---- on-dog fixes (ChongQing rigs, ported from the field version) ----
        # Retime FP stamps onto the HOST clock: the VRTK's clock can drift or
        # jump vs the dog's (PTP-disciplined) clock, while the lidar stamps —
        # and therefore the shared-stamp triplet — live on the host axis.
        # stamp += sliding-median(host_now − fp_stamp); a residual jump beyond
        # retime_jump_sec resets the estimate (VRTK2 time jump). Relay mode only.
        self.declare_parameter("retime_to_host_clock", True)
        self.declare_parameter("retime_jump_sec", 0.5)
        # OEM strips /ODOM covariance to zero — downstream expects that.
        self.declare_parameter("zero_covariance", True)
        # ---- /LOC_BODY_POINTS ----
        self.declare_parameter("lidar_input_topic", "/LIDAR/POINTS")
        self.declare_parameter("loc_body_points_topic", "/LOC_BODY_POINTS")
        self.declare_parameter("loc_body_frame_id", "base_link")
        self.declare_parameter("imu_topic", "/IMU")
        self.declare_parameter("deskew_enable", True)
        self.declare_parameter("deskew_use_linear_velocity", True)
        # lidar→base extrinsic (x y z roll pitch yaw); identity on the M20 URDF
        self.declare_parameter("lidar_to_base_xyzrpy", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # ---- /LOCATION_STATUS ----
        self.declare_parameter("location_status_topic", "/LOCATION_STATUS")
        # fallback status rate while the lidar is silent (cloud-synced otherwise)
        self.declare_parameter("location_status_rate_hz", 10.0)
        self.declare_parameter("leg_odom_topic", "/MOTION_INFO")
        # matching health source (fslam-mode container odom). "" disables →
        # matching_error always normal (fixposition-only mode has no matching).
        self.declare_parameter("matching_odom_topic", "")
        self.declare_parameter("lidar_timeout_sec", 0.5)
        self.declare_parameter("imu_timeout_sec", 0.3)
        self.declare_parameter("leg_odom_timeout_sec", 0.5)
        self.declare_parameter("rtk_timeout_sec", 1.0)
        self.declare_parameter("matching_timeout_sec", 0.5)
        self.declare_parameter("error_hold_sec", 5.0)
        self.declare_parameter("overtime_threshold_sec", 0.08)
        self.declare_parameter("timestamp_skew_threshold_sec", 1.0)

        self.relay_enable = bool(self.get_parameter("relay_enable").value)
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        input_depth = int(self.get_parameter("input_depth").value)
        output_depth = int(self.get_parameter("output_depth").value)
        self.target_child_frame_id = self.get_parameter("target_child_frame_id").value
        rtk_topic = self.get_parameter("rtk_topic").value
        self.retime_enable = bool(self.get_parameter("retime_to_host_clock").value) and self.relay_enable
        self.retime_jump = float(self.get_parameter("retime_jump_sec").value)
        self.zero_covariance = bool(self.get_parameter("zero_covariance").value)
        self._retime_dbuf = deque(maxlen=50)  # host_now − fp_stamp samples
        lidar_topic = self.get_parameter("lidar_input_topic").value
        body_points_topic = self.get_parameter("loc_body_points_topic").value
        self.loc_body_frame_id = self.get_parameter("loc_body_frame_id").value
        imu_topic = self.get_parameter("imu_topic").value
        self.deskew_enable = bool(self.get_parameter("deskew_enable").value) and np is not None
        self.deskew_use_vel = bool(self.get_parameter("deskew_use_linear_velocity").value)
        xyzrpy = [float(v) for v in self.get_parameter("lidar_to_base_xyzrpy").value]
        status_topic = self.get_parameter("location_status_topic").value
        status_rate = float(self.get_parameter("location_status_rate_hz").value)
        leg_odom_topic = self.get_parameter("leg_odom_topic").value
        matching_topic = self.get_parameter("matching_odom_topic").value
        self.timeouts = {
            "lidar": float(self.get_parameter("lidar_timeout_sec").value),
            "imu": float(self.get_parameter("imu_timeout_sec").value),
            "leg_odom": float(self.get_parameter("leg_odom_timeout_sec").value),
            "rtk": float(self.get_parameter("rtk_timeout_sec").value),
            "matching": float(self.get_parameter("matching_timeout_sec").value),
        }
        self.error_hold_sec = float(self.get_parameter("error_hold_sec").value)
        self.overtime_threshold = float(self.get_parameter("overtime_threshold_sec").value)
        self.stamp_skew_threshold = float(self.get_parameter("timestamp_skew_threshold_sec").value)

        self._extrinsic = self._make_extrinsic(xyzrpy)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=input_depth,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=output_depth,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- shared state (executor thread + deskew worker thread) ----
        # Single-item assignments on these dicts are atomic in CPython; the
        # lists and the newest-odom tuple are guarded by explicit locks.
        # name → last receive time (time.monotonic()); None = never seen
        self._last_rx = {k: None for k in ("lidar", "imu", "leg_odom", "rtk", "matching")}
        # sticky error events: name → monotonic time of last occurrence
        self._last_err = {k: None for k in ("overtime", "numeric", "timestamp", "pub")}
        self._imu_lock = threading.Lock()
        self._imu_t = []  # gyro history for deskew (seconds, message stamps)
        self._imu_w = []
        self._imu_keep_sec = 2.0
        self._state_lock = threading.Lock()
        # newest relayed odometry: (stamp_sec, stamp_nsec, t_float, vel_body)
        self._newest_odom = None
        self._initialized = False  # first valid pose received → leaves state 0
        self._last_odom_out_mono = None
        self._last_cloud_status_mono = None
        self._status_seq = 0
        self._status_period = 1.0 / status_rate if status_rate > 0.0 else 0.1

        # latest-scan-wins hand-off to the deskew worker
        self._cloud_cond = threading.Condition()
        self._pending_cloud = None
        self._dropped_clouds = 0
        self._stop_worker = False

        # ---- /ODOM (highest priority — handled inline, never waits) ----
        source_topic = input_topic if self.relay_enable else output_topic
        self.publisher_ = self.create_publisher(Odometry, output_topic, pub_qos) if self.relay_enable else None
        self.subscription_ = self.create_subscription(Odometry, source_topic, self.odom_cb, sub_qos)
        self.message_count_ = 0
        # rtk liveness piggybacks on the source subscription when the topics match
        self._rtk_via_source = bool(rtk_topic) and rtk_topic == source_topic
        self.rtk_sub_ = None
        if rtk_topic and not self._rtk_via_source:
            self.rtk_sub_ = self.create_subscription(
                Odometry, rtk_topic, lambda m: self._mark_rx("rtk"), sensor_qos
            )
        if self.relay_enable:
            self.get_logger().info(
                f"Relaying {input_topic} (best_effort) -> {output_topic} (reliable, newest-first), "
                f"child_frame_id={self.target_child_frame_id}"
            )
        else:
            self.get_logger().info(
                f"Relay disabled — {output_topic} is published directly by the fslam pipeline; "
                f"tracking it as the pose/stamp source"
            )

        # ---- /LOC_BODY_POINTS ----
        self.body_points_pub_ = self.create_publisher(PointCloud2, body_points_topic, sensor_qos)
        self.lidar_sub_ = self.create_subscription(PointCloud2, lidar_topic, self.cloud_cb, sensor_qos)
        self.imu_sub_ = self.create_subscription(Imu, imu_topic, self.imu_cb, sensor_qos)
        self.cloud_count_ = 0
        if np is None:
            self.get_logger().warn("numpy not available — /LOC_BODY_POINTS degrades to a frame relay (no deskew)")
        self.get_logger().info(
            f"Deskewing {lidar_topic} -> {body_points_topic} (frame {self.loc_body_frame_id}, "
            f"gyro {imu_topic}, deskew={'on' if self.deskew_enable else 'off'}, ref = newest /ODOM stamp)"
        )

        # ---- /LOCATION_STATUS ----
        self.matching_sub_ = None
        if matching_topic:
            self.matching_sub_ = self.create_subscription(
                Odometry, matching_topic, lambda m: self._mark_rx("matching"), sensor_qos
            )
        self.matching_enabled_ = bool(matching_topic)
        self.status_pub_ = None
        if HAVE_DRDDS:
            self.status_pub_ = self.create_publisher(LocationStatus, status_topic, pub_qos)
            self.leg_odom_sub_ = self.create_subscription(
                MotionInfo, leg_odom_topic, lambda m: self._mark_rx("leg_odom"), sensor_qos
            )
            self.status_timer_ = self.create_timer(self._status_period, self._status_timer_cb)
            self.get_logger().info(
                f"Publishing {status_topic} cloud-synced (fallback timer {status_rate:g} Hz)"
            )
        else:
            self.get_logger().warn("drdds not importable — /LOCATION_STATUS and leg-odom monitor disabled")

        self._worker = threading.Thread(target=self._cloud_worker, name="deskew_worker", daemon=True)
        self._worker.start()

    def stop(self):
        with self._cloud_cond:
            self._stop_worker = True
            self._cloud_cond.notify()
        self._worker.join(timeout=2.0)

    # ------------------------------------------------------------------ utils

    @staticmethod
    def _make_extrinsic(xyzrpy):
        """(R, t) lidar→base, or None when identity (the M20 default)."""
        if np is None or all(abs(v) < 1e-12 for v in xyzrpy):
            return None
        x, y, z, roll, pitch, yaw = xyzrpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rot = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        return rot, np.array([x, y, z])

    def _mark_rx(self, name):
        self._last_rx[name] = time.monotonic()

    def _mark_err(self, name):
        self._last_err[name] = time.monotonic()

    def _fresh(self, name):
        last = self._last_rx[name]
        return last is not None and (time.monotonic() - last) <= self.timeouts[name]

    def _err_active(self, name):
        last = self._last_err[name]
        return last is not None and (time.monotonic() - last) <= self.error_hold_sec

    def _now_epoch(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _retime(self, sec, nsec):
        """Translate an FP stamp onto the host clock axis (see param docs)."""
        fp = sec + nsec * 1e-9
        d = self._now_epoch() - fp
        if self._retime_dbuf and abs(d - statistics.median(self._retime_dbuf)) > self.retime_jump:
            self._retime_dbuf.clear()  # FP clock jumped — re-converge
            self._mark_err("timestamp")
        self._retime_dbuf.append(d)
        t = fp + statistics.median(self._retime_dbuf)
        out_sec = int(t)
        out_nsec = int(round((t - out_sec) * 1e9))
        if out_nsec >= 1_000_000_000:
            out_sec += 1
            out_nsec -= 1_000_000_000
        return out_sec, out_nsec

    # ------------------------------------------------- executor-side callbacks

    def odom_cb(self, msg: Odometry):
        if self._rtk_via_source:
            self._mark_rx("rtk")
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        lv, av = msg.twist.twist.linear, msg.twist.twist.angular
        vals = (p.x, p.y, p.z, o.x, o.y, o.z, o.w, lv.x, lv.y, lv.z, av.x, av.y, av.z)
        if not all(math.isfinite(v) for v in vals):
            self._mark_err("numeric")
            self.get_logger().warn("Non-finite value in source odometry — message dropped", throttle_duration_sec=5.0)
            return

        sec, nsec = msg.header.stamp.sec, msg.header.stamp.nanosec
        if self.retime_enable:
            sec, nsec = self._retime(sec, nsec)

        if self.relay_enable:
            out = Odometry()
            out.header.frame_id = msg.header.frame_id
            out.header.stamp.sec = sec
            out.header.stamp.nanosec = nsec
            out.child_frame_id = self.target_child_frame_id
            out.pose.pose = msg.pose.pose
            out.twist.twist = msg.twist.twist
            if not self.zero_covariance:
                out.pose.covariance = msg.pose.covariance
                out.twist.covariance = msg.twist.covariance
            try:
                self.publisher_.publish(out)
            except Exception as exc:  # pragma: no cover
                self._mark_err("pub")
                self.get_logger().error(f"Failed to publish odometry: {exc}", throttle_duration_sec=5.0)
                return

        t = sec + nsec * 1e-9
        vel = np.array([lv.x, lv.y, lv.z]) if np is not None else None
        with self._state_lock:
            self._initialized = True
            self._newest_odom = (sec, nsec, t, vel)
        # relay mode: we just published /ODOM; tracker mode: we just RECEIVED it —
        # either way /ODOM is verifiably flowing.
        self._last_odom_out_mono = time.monotonic()

        self.message_count_ += 1
        verb = "relayed" if self.relay_enable else "tracked"
        if self.message_count_ == 1:
            self.get_logger().info(f"First odometry message {verb}")
        elif self.message_count_ % 200 == 0:
            self.get_logger().info(f"{verb.capitalize()} {self.message_count_} odometry messages")

    def imu_cb(self, msg: Imu):
        self._mark_rx("imu")
        w = msg.angular_velocity
        if not (math.isfinite(w.x) and math.isfinite(w.y) and math.isfinite(w.z)):
            self._mark_err("numeric")
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._imu_lock:
            self._imu_t.append(t)
            self._imu_w.append((w.x, w.y, w.z))
            cutoff = t - self._imu_keep_sec
            while self._imu_t and self._imu_t[0] < cutoff:
                self._imu_t.pop(0)
                self._imu_w.pop(0)

    def cloud_cb(self, msg: PointCloud2):
        """Cheap hand-off only — the deskew runs in the worker thread."""
        self._mark_rx("lidar")
        t_scan = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(self._now_epoch() - t_scan) > self.stamp_skew_threshold:
            self._mark_err("timestamp")
        with self._cloud_cond:
            if self._pending_cloud is not None:
                self._dropped_clouds += 1
                self.get_logger().warn(
                    f"Deskew backlog — dropped {self._dropped_clouds} older scan(s)",
                    throttle_duration_sec=10.0,
                )
            self._pending_cloud = msg
            self._cloud_cond.notify()

    # ----------------------------------------------------- deskew worker side

    def _cloud_worker(self):
        while True:
            with self._cloud_cond:
                while self._pending_cloud is None and not self._stop_worker:
                    self._cloud_cond.wait()
                if self._stop_worker:
                    return
                msg = self._pending_cloud
                self._pending_cloud = None
            try:
                self._process_cloud(msg)
            except Exception as exc:  # pragma: no cover — worker must survive
                self._mark_err("pub")
                self.get_logger().error(f"Cloud processing failed: {exc}", throttle_duration_sec=5.0)

    def _process_cloud(self, msg: PointCloud2):
        t0 = time.monotonic()
        t_scan = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Reference = newest /ODOM stamp, so the cloud (and status) share the
        # stamp of a real /ODOM message. Falls back to the scan stamp before
        # the first odometry or when the two clocks disagree wildly.
        with self._state_lock:
            newest = self._newest_odom
        ref_sec, ref_nsec, t_ref = msg.header.stamp.sec, msg.header.stamp.nanosec, t_scan
        if newest is not None:
            n_sec, n_nsec, n_t, _ = newest
            if abs(n_t - t_scan) <= self.stamp_skew_threshold:
                ref_sec, ref_nsec, t_ref = n_sec, n_nsec, n_t
            else:
                self._mark_err("timestamp")  # odom/lidar clocks disagree

        out = PointCloud2()
        out.header.stamp.sec = ref_sec
        out.header.stamp.nanosec = ref_nsec
        out.header.frame_id = self.loc_body_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense

        deskewed = False
        if self.deskew_enable:
            data = self._deskew(msg, t_scan, t_ref)
            if data is not None:
                out.data = data
                deskewed = True
        if not deskewed:
            out.data = msg.data

        elapsed = time.monotonic() - t0
        if elapsed > self.overtime_threshold:
            self._mark_err("overtime")
        try:
            self.body_points_pub_.publish(out)
        except Exception as exc:  # pragma: no cover
            self._mark_err("pub")
            self.get_logger().error(f"Failed to publish body points: {exc}", throttle_duration_sec=5.0)

        # /LOCATION_STATUS shares the same stamp as the cloud (= newest /ODOM).
        if self.status_pub_ is not None:
            self._publish_status_raw(ref_sec, ref_nsec)
            self._last_cloud_status_mono = time.monotonic()

        self.cloud_count_ += 1
        if self.cloud_count_ == 1:
            self.get_logger().info(
                f"First cloud published ({msg.width * msg.height} pts, deskew={'yes' if deskewed else 'NO'}, "
                f"{elapsed * 1e3:.1f} ms, ref-scan offset {(t_ref - t_scan) * 1e3:.0f} ms)"
            )
        elif self.cloud_count_ % 600 == 0:
            self.get_logger().info(f"Published {self.cloud_count_} clouds (last {elapsed * 1e3:.1f} ms)")

    def _deskew(self, msg: PointCloud2, t_scan, t_ref):
        """Return deskewed serialized point data, or None to fall back to relay."""
        dtype, time_field = build_cloud_dtype(msg.fields, msg.point_step)
        if dtype is None or time_field is None:
            self.get_logger().warn("Cloud lacks xyz/time fields — relaying without deskew", throttle_duration_sec=10.0)
            return None
        n = msg.width * msg.height
        if n == 0:
            return None
        buf = bytearray(msg.data)
        arr = np.frombuffer(buf, dtype=dtype, count=n)
        rel_scan = relative_point_times(arr[time_field], t_scan)
        if rel_scan is None:
            self.get_logger().warn("Unrecognized per-point time layout — relaying without deskew", throttle_duration_sec=10.0)
            return None
        rel_t = rel_scan + (t_scan - t_ref)  # seconds relative to the /ODOM ref
        with self._imu_lock:
            if not self._imu_t:
                return None
            imu_t = np.array(self._imu_t)
            imu_w = np.array(self._imu_w)
        with self._state_lock:
            newest = self._newest_odom
        vel = newest[3] if (newest is not None and self.deskew_use_vel) else None
        xyz = np.empty((n, 3), dtype=np.float32)
        xyz[:, 0] = arr['x']
        xyz[:, 1] = arr['y']
        xyz[:, 2] = arr['z']
        if self._extrinsic is not None:
            rot, trans = self._extrinsic
            xyz = (xyz @ rot.T.astype(np.float32)) + trans.astype(np.float32)
        out_xyz = deskew_points(xyz, rel_t, imu_t, imu_w, t_ref, vel)
        arr['x'] = out_xyz[:, 0]
        arr['y'] = out_xyz[:, 1]
        arr['z'] = out_xyz[:, 2]
        return bytes(buf)

    # ---------------------------------------------------------------- status

    def _status_timer_cb(self):
        """Fallback: keep /LOCATION_STATUS flowing while the lidar is silent."""
        if self._last_cloud_status_mono is not None:
            if time.monotonic() - self._last_cloud_status_mono < 1.5 * self._status_period:
                return
        with self._state_lock:
            newest = self._newest_odom
        if newest is not None:  # still share the newest /ODOM stamp
            sec, nsec = newest[0], newest[1]
        else:
            now_ns = self.get_clock().now().nanoseconds
            sec, nsec = int(now_ns // 1_000_000_000), int(now_ns % 1_000_000_000)
        self._publish_status_raw(sec, nsec)

    def _publish_status_raw(self, sec, nsec):
        # Real drdds layout (verified on-dog): LocationStatus.meta is a MetaType
        # {uint64 frame_id, builtin_interfaces/Time stamp} — note `meta`, NOT
        # `header` (MotionInfo DOES use `header`; OEM naming is inconsistent).
        msg = LocationStatus()
        self._status_seq += 1
        msg.meta.frame_id = self._status_seq
        msg.meta.stamp.sec = sec
        msg.meta.stamp.nanosec = nsec

        iv = msg.data.input_val
        iv.lidar = INPUT_OK if self._fresh("lidar") else INPUT_MISSING
        iv.imu = INPUT_OK if self._fresh("imu") else INPUT_MISSING
        iv.leg_odom = INPUT_OK if self._fresh("leg_odom") else INPUT_MISSING
        iv.rtk = INPUT_OK if self._fresh("rtk") else INPUT_MISSING
        iv.tag = INPUT_MISSING  # not equipped on this deployment
        iv.reflective_strip = INPUT_MISSING  # not equipped on this deployment

        odom_flowing = (
            self._last_odom_out_mono is not None
            and (time.monotonic() - self._last_odom_out_mono) <= self.timeouts["rtk"]
        )
        lv = msg.data.loss_val
        lv.odom_error = ERR_OK if odom_flowing else ERR_ERROR
        lv.leg_odom_error = ERR_OK if self._fresh("leg_odom") else ERR_ERROR
        if self.matching_enabled_:
            lv.matching_error = ERR_OK if self._fresh("matching") else ERR_ERROR
        else:
            lv.matching_error = ERR_OK  # no scan matching in fixposition-only mode
        lv.pub_error = ERR_ERROR if self._err_active("pub") else ERR_OK

        ev = msg.data.exec_val
        ev.overtime = ERR_ERROR if self._err_active("overtime") else ERR_OK
        ev.numeric_error = ERR_ERROR if self._err_active("numeric") else ERR_OK
        ev.timestamp_error = ERR_ERROR if self._err_active("timestamp") else ERR_OK

        # total_status per OEM spec: 0 uninit / 1 normal / 2 low quality / 3 lost.
        # "Low quality" here is liveness-based (degraded inputs / exec errors);
        # a matching-quality metric (inlier_ratio) can refine this in the C++ port.
        with self._state_lock:
            initialized = self._initialized
        if not initialized:
            total = TOTAL_UNINITIALIZED
        elif not odom_flowing or ev.numeric_error == ERR_ERROR:
            total = TOTAL_LOST
        elif (
            ev.overtime == ERR_ERROR
            or ev.timestamp_error == ERR_ERROR
            or lv.matching_error == ERR_ERROR
            or lv.pub_error == ERR_ERROR
            or iv.lidar == INPUT_MISSING
            or iv.imu == INPUT_MISSING
            or iv.rtk == INPUT_MISSING
            or iv.leg_odom == INPUT_MISSING
        ):
            total = TOTAL_LOW_QUALITY
        else:
            total = TOTAL_NORMAL
        msg.data.total_status = total

        try:
            self.status_pub_.publish(msg)
        except Exception as exc:  # pragma: no cover
            self._mark_err("pub")
            self.get_logger().error(f"Failed to publish location status: {exc}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = FixpositionOdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
