#!/usr/bin/env python3
# ============================================================================
# cloud_relay_out.py —— 点云中继发布端 | publishing side of the cloud relay
# ============================================================================
# 从 stdin 读 cloud_relay_in.py 的 [u32 长度 | payload] 帧,在驱动同款多网卡
# profile(继承容器的 FASTRTPS_DEFAULT_PROFILES_FILE)上以 raw bytes 重发为
# /M20/LIDAR_POINTS。驱动的 m20.lidar_topic 指向这个话题。背景见
# cloud_relay_in.py 头注释。
#
# Reads [u32 length | payload] frames from cloud_relay_in.py on stdin and
# republishes them as raw bytes on /M20/LIDAR_POINTS using the driver's
# multi-interface profile (inherited FASTRTPS_DEFAULT_PROFILES_FILE). The
# driver's m20.lidar_topic points here. Background: cloud_relay_in.py header.
# ============================================================================
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

OUT_TOPIC = "/M20/LIDAR_POINTS"
BEST = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


def main():
    rclpy.init()
    node = Node("m20_cloud_relay_out")
    pub = node.create_publisher(PointCloud2, OUT_TOPIC, BEST)
    node.get_logger().info("republishing stdin frames on %s" % OUT_TOPIC)
    src = sys.stdin.buffer
    try:
        while True:
            hdr = src.read(4)
            if len(hdr) < 4:
                break  # 上游退出(EOF)| upstream ended
            (length,) = struct.unpack("<I", hdr)
            data = src.read(length)
            if len(data) < length:
                break
            pub.publish(data)  # bytes 入参 = 发布 serialized 消息,不反序列化
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
