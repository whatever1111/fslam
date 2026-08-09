#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class FixpositionOdomRelay(Node):
    def __init__(self):
        super().__init__("fixposition_odom_to_odom")

        self.declare_parameter("input_topic", "/fixposition/odometry_enu")
        self.declare_parameter("output_topic", "/ODOM")
        self.declare_parameter("input_depth", 10)
        self.declare_parameter("output_depth", 10)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        input_depth = int(self.get_parameter("input_depth").value)
        output_depth = int(self.get_parameter("output_depth").value)

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

        self.publisher_ = self.create_publisher(Odometry, output_topic, pub_qos)
        self.subscription_ = self.create_subscription(Odometry, input_topic, self.odom_cb, sub_qos)
        self.message_count_ = 0

        self.get_logger().info(f"Relaying {input_topic} (best_effort) -> {output_topic} (reliable)")

    def odom_cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.publisher_.publish(out)

        self.message_count_ += 1
        if self.message_count_ == 1:
            self.get_logger().info("First odometry message relayed")
        elif self.message_count_ % 200 == 0:
            self.get_logger().info(f"Relayed {self.message_count_} odometry messages")


def main(args=None):
    rclpy.init(args=args)
    node = FixpositionOdomRelay()
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
