#!/usr/bin/env python3
import time

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
        # 输出限频:FP 设备的轮速辅助 50Hz 足够,而 /MOTION_INFO 可能是数百 Hz,
        # 逐条转发白烧 CPU。0 = 不限频(旧行为)。
        # Output rate cap: 50 Hz is plenty for FP wheel-speed aiding while
        # /MOTION_INFO can run at hundreds of Hz. 0 = uncapped (legacy behavior).
        self.declare_parameter("max_rate_hz", 50.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.use_y = bool(self.get_parameter("use_y").value)
        self.publish_yaw = bool(self.get_parameter("publish_yaw").value)
        max_rate = float(self.get_parameter("max_rate_hz").value)
        self.min_period_ns = int(1e9 / max_rate) if max_rate > 0.0 else 0
        self.last_pub_ns = 0

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        # sensor QoS(best-effort、浅队列):轮速只有最新值有意义,积压的旧样本
        # 是纯浪费的反序列化;对 reliable/best-effort 两类上游发布者都兼容。
        # sensor QoS (best-effort, shallow queue): only the latest wheel speed
        # matters — a backlog of stale samples is wasted deserialization. This
        # subscription matches both reliable and best-effort upstream publishers.
        self.subscription = self.create_subscription(
            MotionInfo, input_topic, self.motion_info_cb, qos_profile_sensor_data
        )
        self.message_count = 0
        self.get_logger().info(f"Bridging {input_topic} -> {output_topic} (max {max_rate:g} Hz)")

    def motion_info_cb(self, msg: MotionInfo):
        if self.min_period_ns:
            now_ns = time.monotonic_ns()
            if now_ns - self.last_pub_ns < self.min_period_ns:
                return
            self.last_pub_ns = now_ns
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
