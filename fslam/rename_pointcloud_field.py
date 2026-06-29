#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class RenamePointCloudField(Node):
    def __init__(self):
        super().__init__("rename_pointcloud_field")
        self.declare_parameter("input_topic", "/LIDAR/POINTS")
        self.declare_parameter("output_topic", "/LIDAR/POINTS_REMAP")
        self.declare_parameter("from_field", "timestamp")
        self.declare_parameter("to_field", "time")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.from_field = str(self.get_parameter("from_field").value)
        self.to_field = str(self.get_parameter("to_field").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self.subscription = self.create_subscription(PointCloud2, input_topic, self.cloud_cb, qos)

        self.message_count = 0
        self.converted_count = 0
        self.layout_key = None
        self.layout = None
        self.warned_missing_field = False
        self.warned_bad_source_field = False

        self.le_unpack_f64 = struct.Struct("<d")
        self.le_pack_f32 = struct.Struct("<f")
        self.be_unpack_f64 = struct.Struct(">d")
        self.be_pack_f32 = struct.Struct(">f")

        self.get_logger().info(
            f"Remapping {input_topic} -> {output_topic} ({self.from_field}:float64 -> {self.to_field}:float32)"
        )

    def _build_layout(self, msg: PointCloud2):
        source_field = None
        output_fields = []
        offset_shift = 0

        for field in msg.fields:
            out_field = PointField()
            out_field.name = field.name
            out_field.offset = field.offset - offset_shift
            out_field.datatype = field.datatype
            out_field.count = field.count

            if field.name == self.from_field:
                source_field = field
                out_field.name = self.to_field
                out_field.datatype = PointField.FLOAT32
                offset_shift = 4

            output_fields.append(out_field)

        if source_field is None:
            return None

        if source_field.datatype != PointField.FLOAT64 or source_field.count != 1:
            return False

        return {
            "source_offset": source_field.offset,
            "source_end": source_field.offset + 8,
            "output_fields": output_fields,
            "output_point_step": msg.point_step - 4,
        }

    def _get_layout(self, msg: PointCloud2):
        layout_key = (
            msg.point_step,
            msg.is_bigendian,
            tuple((field.name, field.offset, field.datatype, field.count) for field in msg.fields),
        )
        if layout_key != self.layout_key:
            self.layout_key = layout_key
            self.layout = self._build_layout(msg)
        return self.layout

    def _set_header_stamp_from_seconds(self, header, timestamp_seconds: float):
        sec = math.floor(timestamp_seconds)
        nanosec = int(round((timestamp_seconds - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec

    def cloud_cb(self, msg: PointCloud2):
        self.message_count += 1
        layout = self._get_layout(msg)

        if layout is None:
            if not self.warned_missing_field:
                self.get_logger().warning(f"Field '{self.from_field}' not found; passing cloud through unchanged")
                self.warned_missing_field = True
            self.publisher.publish(msg)
            return

        if layout is False:
            if not self.warned_bad_source_field:
                self.get_logger().warning(
                    f"Field '{self.from_field}' is not float64[count=1]; passing cloud through unchanged"
                )
                self.warned_bad_source_field = True
            self.publisher.publish(msg)
            return

        row_padding = msg.row_step - (msg.width * msg.point_step)
        output_point_step = layout["output_point_step"]
        output_row_step = (msg.width * output_point_step) + row_padding
        output_data = bytearray(output_row_step * msg.height)

        source_offset = layout["source_offset"]
        source_end = layout["source_end"]
        suffix_length = msg.point_step - source_end

        unpack_f64 = self.be_unpack_f64.unpack_from if msg.is_bigendian else self.le_unpack_f64.unpack_from
        pack_f32 = self.be_pack_f32.pack_into if msg.is_bigendian else self.le_pack_f32.pack_into

        src = memoryview(msg.data)
        dst = memoryview(output_data)

        min_time_value = None
        max_time_value = None
        for row in range(msg.height):
            src_row_base = row * msg.row_step
            for col in range(msg.width):
                src_point_base = src_row_base + (col * msg.point_step)
                time_value = unpack_f64(src, src_point_base + source_offset)[0]
                if min_time_value is None or time_value < min_time_value:
                    min_time_value = time_value
                if max_time_value is None or time_value > max_time_value:
                    max_time_value = time_value

        if min_time_value is None or max_time_value is None:
            self.publisher.publish(msg)
            return

        max_relative_time = 0.0
        for row in range(msg.height):
            src_row_base = row * msg.row_step
            dst_row_base = row * output_row_step

            for col in range(msg.width):
                src_point_base = src_row_base + (col * msg.point_step)
                dst_point_base = dst_row_base + (col * output_point_step)

                if source_offset:
                    dst[dst_point_base : dst_point_base + source_offset] = src[
                        src_point_base : src_point_base + source_offset
                    ]

                relative_time_value = unpack_f64(src, src_point_base + source_offset)[0] - min_time_value
                if relative_time_value > max_relative_time:
                    max_relative_time = relative_time_value
                pack_f32(output_data, dst_point_base + source_offset, relative_time_value)

                if suffix_length:
                    dst[dst_point_base + source_offset + 4 : dst_point_base + output_point_step] = src[
                        src_point_base + source_end : src_point_base + msg.point_step
                    ]

            if row_padding:
                dst[dst_row_base + (msg.width * output_point_step) : dst_row_base + output_row_step] = src[
                    src_row_base + (msg.width * msg.point_step) : src_row_base + msg.row_step
                ]

        out = PointCloud2()
        out.header = msg.header
        self._set_header_stamp_from_seconds(out.header, max_time_value)
        out.height = msg.height
        out.width = msg.width
        out.fields = layout["output_fields"]
        out.is_bigendian = msg.is_bigendian
        out.point_step = output_point_step
        out.row_step = output_row_step
        out.data = output_data
        out.is_dense = msg.is_dense

        self.publisher.publish(out)

        self.converted_count += 1
        if self.converted_count == 1:
            self.get_logger().info(
                f"Published first converted cloud (relative time range: 0.0 .. {max_relative_time:.6f}s, header stamp: {out.header.stamp.sec}.{out.header.stamp.nanosec:09d})"
            )
        elif self.converted_count % 2000 == 0:
            self.get_logger().info(
                f"Published {self.message_count} clouds ({self.converted_count} converted)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RenamePointCloudField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
