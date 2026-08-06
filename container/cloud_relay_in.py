#!/usr/bin/env python3
# ============================================================================
# cloud_relay_in.py —— /LIDAR/POINTS 环回接收端 | loopback-side of the cloud relay
# ============================================================================
# OEM 雷达驱动把点云 writer 放在一个只绑 127.0.0.1 的 DDS participant 上
# (whitelist 只有环回,发不了真实网卡)。FastDDS 2.6(Humble)的 participant 一旦
# 带真实网卡就不再通告环回 locator,对端因此永远回不了包 —— 多网卡 profile 实测
# 永远发现不了这个 writer,只有"纯环回 participant"能收到(Foxy 2.0 没这问题,
# 所以老的宿主机 Python 栈从来没撞过这堵墙)。
#
# 解法:本进程用纯环回 profile(fastdds_lo.xml)订阅 /LIDAR/POINTS,把原始
# serialized 字节按 [u32 长度 | payload] 写到 stdout;cloud_relay_out.py 在驱动
# 的多网卡 profile 上原样重发。两个 DDS 世界之间用管道桥接,不走 DDS。
#
# The OEM lidar driver puts its cloud writer on a DDS participant bound to
# 127.0.0.1 only. FastDDS 2.6 (Humble) stops announcing a loopback locator once
# a participant has real interfaces, so the OEM side can never answer it — a
# multi-interface profile never discovers this writer; only a loopback-only
# participant receives it (Foxy 2.0 announced loopback alongside real NICs,
# which is why the old host-side Python stack never hit this wall).
#
# This process subscribes with the loopback-only profile (fastdds_lo.xml) and
# streams the raw serialized bytes as [u32 length | payload] frames to stdout;
# cloud_relay_out.py republishes them on the driver's multi-interface profile.
# The two DDS worlds are bridged by a pipe, not by DDS.
#
# 用法 | usage (entrypoint_m20.sh):
#   FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_lo.xml python3 cloud_relay_in.py \
#     | python3 cloud_relay_out.py
# ============================================================================
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

LIDAR_TOPIC = "/LIDAR/POINTS"
BEST = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


def main():
    rclpy.init()
    node = Node("m20_cloud_relay_in")
    out = sys.stdout.buffer

    def cb(raw):
        # raw=True 订阅直接拿 serialized 字节,不做反序列化 —— ~1 MB @10 Hz 零拷贝转发。
        # raw=True hands us the serialized bytes; no deserialize on the hot path.
        out.write(struct.pack("<I", len(raw)))
        out.write(raw)
        out.flush()

    node.create_subscription(PointCloud2, LIDAR_TOPIC, cb, BEST, raw=True)
    node.get_logger().info("relaying %s (loopback participant) to stdout" % LIDAR_TOPIC)
    try:
        rclpy.spin(node)
    except (BrokenPipeError, KeyboardInterrupt):
        pass  # 对端退出/收到中断:安静退出,entrypoint 的 respawn 循环会拉起整条管道
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
