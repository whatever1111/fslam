#!/usr/bin/env python3
"""lidar_stamp_probe.py — /LIDAR/POINTS header.stamp vs per-point absolute timestamps, live.

The M20 OEM RoboSense feed flips its header-stamp semantics mid-stream (mode B:
header = first point; mode A: header = last point + 13 ms; 22 %/16 % of frames in the
2026-08-14/13 bags, all in motion segments; 0 flips in 300 s at rest). Run this during a
drive to see the live mode fraction. Read-only, one best-effort subscription.

  python3 lidar_stamp_probe.py [topic] [duration_s|0=until SIGINT] [out.csv]

Prints a summary at the end (also on SIGINT). Root is required on the dog (SHM UID wall).
"""
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/LIDAR/POINTS'
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
OUT = sys.argv[3] if len(sys.argv) > 3 else '/tmp/lidar_stamp_probe.csv'
rows = []
stop = False


def on_sig(_s, _f):
    global stop
    stop = True


class Probe(Node):
    def __init__(self):
        super().__init__('lidar_stamp_probe')
        self.sub = self.create_subscription(PointCloud2, TOPIC, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        rt = time.time()
        hs = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        off = [f.offset for f in m.fields if f.name == 'timestamp']
        step = m.point_step
        n = len(m.data) // step
        if not off or n == 0:
            rows.append((hs, rt, float('nan'), float('nan'), n))
            return
        buf = np.frombuffer(m.data, dtype=np.uint8)[:n * step].reshape(n, step)
        ts = np.frombuffer(np.ascontiguousarray(buf[:, off[0]:off[0] + 8]).tobytes(), dtype='<f8')
        ts = ts[np.isfinite(ts) & (ts > 1e9)]
        rows.append((hs, rt, ts.min() if len(ts) else float('nan'), ts.max() if len(ts) else float('nan'), n))


def summary(a):
    if len(a) == 0:
        print(f'[LIDAR-STAMP] NO MESSAGES on {TOPIC}')
        return
    d = a[:, 2] - a[:, 0]
    mode_a = d < -0.05
    dur = a[-1, 1] - a[0, 1]
    print(f'[LIDAR-STAMP] topic={TOPIC} msgs={len(a)} dur={dur:.1f}s rate={len(a) / max(1e-9, dur):.2f}Hz')
    print(f'[LIDAR-STAMP] tmin-hdr median {np.nanmedian(d):+.4f} p5 {np.nanpercentile(d, 5):+.4f} '
          f'p95 {np.nanpercentile(d, 95):+.4f} s')
    print(f'[LIDAR-STAMP] mode A (hdr = end-of-scan+13ms) {mode_a.mean() * 100:.1f}%   '
          f'mode B (hdr = first point) {(~mode_a).mean() * 100:.1f}%')
    print(f'[LIDAR-STAMP] recv-tmax median {np.nanmedian(a[:, 1] - a[:, 3]):+.4f}  '
          f'span median {np.nanmedian(a[:, 3] - a[:, 2]):.4f}')
    ch = np.where(np.diff(mode_a.astype(int)) != 0)[0]
    t0 = a[0, 1]
    print(f'[LIDAR-STAMP] flips={len(ch)} at t=' + ', '.join(f'{a[i + 1, 1] - t0:.1f}' for i in ch[:40]))
    if len(ch):
        segs = np.split(np.arange(len(a)), ch + 1)
        print('[LIDAR-STAMP] segments: ' + ' '.join(
            f"{'A' if mode_a[s[0]] else 'B'}[{a[s[0], 1] - t0:.0f}-{a[s[-1], 1] - t0:.0f}s]" for s in segs[:40]))
    print(f'[LIDAR-STAMP] VERDICT: ' + ('FLIP CONFIRMED LIVE' if len(ch) else 'no flip in this sample'))


def main():
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)
    rclpy.init()
    node = Probe()
    end = time.time() + DUR if DUR > 0 else float('inf')
    while rclpy.ok() and not stop and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    a = np.array(rows, dtype=float).reshape(-1, 5)
    np.savetxt(OUT, a, delimiter=',', header='hdr,recv,tmin,tmax,n')
    summary(a)


if __name__ == '__main__':
    main()
