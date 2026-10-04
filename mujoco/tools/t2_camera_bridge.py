"""Publish NUSim camera shared memory on FC Isar's ROS 2 T2 topics.

Run in the same Linux IPC namespace as the simulator. The camera extrinsics
and fovy in the T2 MJCF are provisional until measured on the physical T2.
"""

from __future__ import annotations

import mmap
import os
import struct
import time


HEADER = struct.Struct("<IIQIII32sffffff4x")  # SharedImageHeader, 88 bytes on x86_64
MAGIC = 0x4E42494D


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Pose, PoseStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    class Bridge(Node):
        def __init__(self):
            super().__init__("fcsim_t2_camera")
            self.image_pub = self.create_publisher(Image, "/boostercamera/head/rgb", 10)
            self.info_pub = self.create_publisher(
                CameraInfo, "/boostercamera/head/rgb/camera_info", 10
            )
            pose_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pose_pub = self.create_publisher(PoseStamped, "/head_pose_stamped", pose_qos)
            self.create_subscription(Pose, "/head_pose", self.on_pose, 10)
            self.mapping = None
            self.last_sequence = 0
            self.last_frame_time = time.monotonic()
            self.create_timer(1.0 / 30.0, self.publish_frame)

        def on_pose(self, pose):
            stamped = PoseStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = "base_footprint"
            stamped.pose = pose
            self.pose_pub.publish(stamped)

        def open_mapping(self):
            try:
                fd = os.open("/dev/shm/_boostercamera_head_rgb", os.O_RDONLY)
                try:
                    self.mapping = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
                finally:
                    os.close(fd)
            except FileNotFoundError:
                return

        def publish_frame(self):
            if self.mapping is None:
                self.open_mapping()
                if self.mapping is None:
                    return
            memory = self.mapping
            try:
                header = HEADER.unpack_from(memory)
            except (ValueError, OSError):
                self.mapping = None
                return
            magic, version, sequence, size, width, height, encoding, focal_norm, _, cx, cy, k1, k2 = header
            if magic != MAGIC or version != 2 or encoding.split(b"\0", 1)[0] != b"rgb8":
                raise RuntimeError("NUSim camera shared-memory layout mismatch")
            if size != width * height * 3 or HEADER.size + size > len(memory):
                raise RuntimeError("Invalid NUSim camera frame size")
            if sequence == self.last_sequence:
                if time.monotonic() - self.last_frame_time > 2.0:
                    memory.close()  # reconnect after a simulator restart
                    self.mapping = None
                    self.last_sequence = 0
                return
            if sequence & 1:
                return
            pixels = bytes(memory[HEADER.size : HEADER.size + size])
            if struct.unpack_from("<Q", memory, 8)[0] != sequence:
                return  # writer raced our copy; retry at next tick
            self.last_sequence = sequence
            self.last_frame_time = time.monotonic()

            stamp = self.get_clock().now().to_msg()
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = "head_camera_optical"
            image.width = width
            image.height = height
            image.encoding = "rgb8"
            image.step = width * 3
            image.data = pixels
            self.image_pub.publish(image)

            fx = focal_norm * width
            fy = fx
            info = CameraInfo()
            info.header = image.header
            info.width = width
            info.height = height
            info.distortion_model = "plumb_bob"
            info.d = [k1, k2, 0.0, 0.0, 0.0]
            info.k = [fx, 0.0, width / 2 + cx, 0.0, fy, height / 2 + cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, width / 2 + cx, 0.0, 0.0, fy, height / 2 + cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        if node.mapping is not None:
            node.mapping.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
