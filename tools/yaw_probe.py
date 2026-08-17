#!/usr/bin/env python3
"""yaw_probe.py — FE yaw (/Odometry) vs Fixposition heading, live, with GNSS status.

Purpose: catch the FE yaw slips at in-place turns that the header-stamp mode flip used to
cause (2026-08-17: −5…−11° after the building with the old adapter, ±1° with
stamp_from_points). The FE heading is in its own frame, so only the CHANGE of
(fe_yaw − fp_yaw) over the drive matters; the FP heading is dual-antenna aided in FIX and
INS-propagated in the outage, so read the drift on FIX stretches, not inside the outage.

  python3 yaw_probe.py [duration_s|0=until SIGINT] [out.csv] [--fe-topic /Odometry]

Subscribes /fixposition/odometry_enu (nav_msgs) for the FP heading + position and, if the
fixposition_driver_msgs Python package is importable, /fixposition/fpa/odomenu for
gnss1_status (8 = FIX). Prints a summary at the end (also on SIGINT).
"""
import math
import signal
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

args = [a for a in sys.argv[1:] if not a.startswith('--')]
DUR = float(args[0]) if len(args) > 0 else 0.0
OUT = args[1] if len(args) > 1 else '/tmp/yaw_probe.csv'
FE_TOPIC = '/Odometry'
if '--fe-topic' in sys.argv:
    FE_TOPIC = sys.argv[sys.argv.index('--fe-topic') + 1]
try:
    from fixposition_driver_msgs.msg import FpaOdomenu  # noqa: E402
    HAVE_FPA = True
except Exception:  # pragma: no cover - depends on the robot's install
    HAVE_FPA = False

stop = False


def on_sig(_s, _f):
    global stop
    stop = True


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Probe(Node):
    def __init__(self):
        super().__init__('yaw_probe')
        self.fe = []   # (t, yaw, x, y)
        self.fp = []   # (t, yaw, x, y)
        self.st = []   # (t, status)
        self.create_subscription(Odometry, FE_TOPIC, self.cb_fe, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fixposition/odometry_enu', self.cb_fp, qos_profile_sensor_data)
        if HAVE_FPA:
            self.create_subscription(FpaOdomenu, '/fixposition/fpa/odomenu', self.cb_st, qos_profile_sensor_data)

    @staticmethod
    def _t(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def cb_fe(self, m):
        p = m.pose.pose.position
        self.fe.append((self._t(m), yaw_of(m.pose.pose.orientation), p.x, p.y))

    def cb_fp(self, m):
        p = m.pose.pose.position
        self.fp.append((self._t(m), yaw_of(m.pose.pose.orientation), p.x, p.y))

    def cb_st(self, m):
        self.st.append((self._t(m), int(m.gnss1_status)))


def summary(node):
    fe = np.array(node.fe, dtype=float).reshape(-1, 4)
    fp = np.array(node.fp, dtype=float).reshape(-1, 4)
    st = np.array(node.st, dtype=float).reshape(-1, 2)
    print(f'[YAW] fe msgs={len(fe)} ({FE_TOPIC})  fp msgs={len(fp)}  status msgs={len(st)}'
          + ('' if HAVE_FPA else '  (fixposition_driver_msgs not importable: no status)'))
    if len(fe) < 20 or len(fp) < 20:
        print('[YAW] not enough data (is the SLAM pipeline running? is the FP driver up?)')
        return
    t = fe[:, 0]
    fp_yaw = np.interp(t, fp[:, 0], np.unwrap(fp[:, 1]))
    fe_yaw = np.unwrap(fe[:, 1])
    diff = np.degrees(np.array([wrap(v) for v in (fe_yaw - fp_yaw)]))
    diff -= np.median(diff[:min(len(diff), 100)])
    fixed = np.ones(len(t), dtype=bool)
    if len(st):
        fixed = np.interp(t, st[:, 0], (st[:, 1] == 8).astype(float)) > 0.99
        print(f'[YAW] FIX fraction {fixed.mean() * 100:.0f}%')
    fpx = np.stack([np.interp(t, fp[:, 0], fp[:, 1 + 1]), np.interp(t, fp[:, 0], fp[:, 1 + 2])], 1)
    d = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(fpx, axis=0), axis=1))])
    print(f'[YAW] travel (FP) {d[-1]:.0f} m over {t[-1] - t[0]:.0f} s')
    print('[YAW] fe−fp yaw offset relative to start, per 60 s (FIX epochs only; NaN = no FIX):')
    t0 = t[0]
    line = []
    for a in np.arange(0, t[-1] - t0 + 1, 60):
        m = (t - t0 >= a) & (t - t0 < a + 60) & fixed
        line.append(f'{a:.0f}s:{np.median(diff[m]):+.1f}°' if m.sum() > 20 else f'{a:.0f}s:nan')
    print('[YAW]   ' + '  '.join(line))
    if fixed.sum() > 40:
        first = np.median(diff[fixed][:200])
        last = np.median(diff[fixed][-200:])
        print(f'[YAW] FIX-epoch drift start→end {last - first:+.2f}° over {d[-1]:.0f} m '
              f'({(last - first) / max(d[-1], 1) * 100:+.2f}°/100 m)  '
              + ('OK (<2°/100 m)' if abs(last - first) / max(d[-1], 1) * 100 < 2 else 'CHECK: yaw drift'))
    np.savetxt(OUT, np.column_stack([t, fe_yaw, fp_yaw, diff, fixed.astype(float), d]), delimiter=',',
               header='t,fe_yaw_rad,fp_yaw_rad,diff_deg_rel,fixed,fp_travel_m')


def main():
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)
    rclpy.init()
    node = Probe()
    end = time.time() + DUR if DUR > 0 else float('inf')
    while rclpy.ok() and not stop and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
    try:
        summary(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
