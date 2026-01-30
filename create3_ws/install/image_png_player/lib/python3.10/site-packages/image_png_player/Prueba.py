#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import os
from glob import glob

class ImagePlayer(Node):
    def __init__(self):
        super().__init__('image_player_rgb_only')

        self.folder = '/home/jlmre/ros2_test/src/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data'
        files = glob(self.folder + '/*.png')
        #self.get_logger().info(f"\n\nFiles:{files}\n\n")

        self.declare_parameter('image_folder', '/home/jlmre/ros2_test/src/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('frame_id', 'camera_link')

        #self.folder = self.get_parameter('image_folder').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_img = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.images = sorted(glob(os.path.join(self.folder, '*.png')) +
                             glob(os.path.join(self.folder, '*.jpg')))
        #self.get_logger().info(f"\n\nNro imagenes: {len(self.images)}\n\n")

        self.index = 0
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / max(self.fps, 0.001), self.timer_cb)

    def timer_cb(self):
        if self.index >= len(self.images):
            self.index = 0  # loop

        fname = self.images[self.index]
        self.index += 1
        img = cv2.imread(fname)
        if img is None:
            return

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.frame_id)

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        ros_img.header = header
        self.pub_img.publish(ros_img)

        ci = CameraInfo()
        ci.header = header
        ci.width = img.shape[1]
        ci.height = img.shape[0]
        w = float(ci.width)
        h = float(ci.height)

        f_x = w  # focal aproximada en píxeles
        f_y = h
        c_x = w / 2.0
        c_y = h / 2.0

        ci.k = [f_x, 0.0, c_x,
                0.0, f_y, c_y,
                0.0, 0.0, 1.0]

        ci.p = [f_x, 0.0, c_x, 0.0,
                0.0, f_y, c_y, 0.0,
                0.0, 0.0, 1.0, 0.0]

        self.pub_info.publish(ci)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
