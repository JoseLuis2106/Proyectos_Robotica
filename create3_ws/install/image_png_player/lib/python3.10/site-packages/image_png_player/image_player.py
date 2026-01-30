#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import os
from glob import glob
from tf2_ros import StaticTransformBroadcaster

class ImagePlayer(Node):
    def __init__(self):
        super().__init__('image_player')

        # Parameters
        self.declare_parameter('image_folder', '/home/jlmre/ros2_test/src/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.folder = self.get_parameter('image_folder').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.pub_img = self.create_publisher(Image, '/rgb/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)

        # TF broadcaster (static)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        # Load images
        self.images = sorted(
            glob(os.path.join(self.folder, '*.png')) +
            glob(os.path.join(self.folder, '*.jpg'))
        )
        self.index = 0

        if not self.images:
            self.get_logger().error("No images found in folder!")

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / max(self.fps, 0.001), self.timer_cb)

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.frame_id

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("Static TF base_link → camera_optical_frame published")

    def timer_cb(self):
        if self.index >= len(self.images):
            return

        fname = self.images[self.index]
        self.index += 1

        img = cv2.imread(fname)
        if img is None:
            return

        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id

        # Publish image
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        ros_img.header = header
        self.pub_img.publish(ros_img)

        # Camera info (aproximate for Euroc dataset)
        ci = CameraInfo()
        ci.header = header
        ci.width = img.shape[1]
        ci.height = img.shape[0]
        ci.k = [458.654,0.0,367.215, 0.0,457.296,248.375, 0.0,0.0,1.0]
        ci.p = [458.654,0.0,367.215,0.0, 0.0,457.296,248.375,0.0, 0.0,0.0,1.0,0.0]
        self.pub_info.publish(ci)

        self.get_logger().info(f"Published image: {fname}")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
