#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__("pointcloud_relay")

        self.declare_parameter("input_topic", "/cloud_registered_body")
        self.declare_parameter("output_topic", "/cloud_registered_body_pgo")
        self.declare_parameter("input_depth", 10)
        self.declare_parameter("output_depth", 10)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        input_depth = int(self.get_parameter("input_depth").value)
        output_depth = int(self.get_parameter("output_depth").value)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=input_depth,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=output_depth,
        )

        self.publisher_ = self.create_publisher(PointCloud2, output_topic, pub_qos)
        self.subscription_ = self.create_subscription(PointCloud2, input_topic, self.cloud_cb, sub_qos)
        self.message_count_ = 0

        self.get_logger().info(f"Relaying {input_topic} (reliable) -> {output_topic} (reliable)")

    def cloud_cb(self, msg: PointCloud2):
        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.publisher_.publish(out)

        self.message_count_ += 1
        if self.message_count_ == 1:
            self.get_logger().info("First point cloud message relayed")
        elif self.message_count_ % 100 == 0:
            self.get_logger().info(f"Relayed {self.message_count_} point cloud messages")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
