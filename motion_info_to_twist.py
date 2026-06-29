#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.node import Node
from drdds.msg import MotionInfo


class MotionInfoToTwist(Node):
    def __init__(self):
        super().__init__("motion_info_to_twist")
        self.declare_parameter("input_topic", "/MOTION_INFO")
        self.declare_parameter("output_topic", "/fixposition/motion_info_twist")
        self.declare_parameter("use_y", False)
        self.declare_parameter("publish_yaw", False)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.use_y = bool(self.get_parameter("use_y").value)
        self.publish_yaw = bool(self.get_parameter("publish_yaw").value)

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(MotionInfo, input_topic, self.motion_info_cb, 10)
        self.message_count = 0
        self.get_logger().info(f"Bridging {input_topic} -> {output_topic}")

    def motion_info_cb(self, msg: MotionInfo):
        twist = Twist()
        twist.linear.x = float(msg.data.vel_x)
        twist.linear.y = float(msg.data.vel_y) if self.use_y else 0.0
        twist.angular.z = float(msg.data.vel_yaw) if self.publish_yaw else 0.0
        self.publisher.publish(twist)

        self.message_count += 1
        if self.message_count == 1:
            self.get_logger().info("Published first Twist converted from MotionInfo")
        elif self.message_count % 20000 == 0:
            self.get_logger().info(f"Published {self.message_count} Twist messages")


def main(args=None):
    rclpy.init(args=args)
    node = MotionInfoToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
