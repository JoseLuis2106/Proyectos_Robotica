#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from tf2_ros import TransformBroadcaster

class MonoVONode(Node):
    def __init__(self):
        super().__init__('mono_vo_node')

        self.declare_parameter('image_topic', '/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')
        self.declare_parameter('max_corners', 300)
        self.declare_parameter('quality_level', 0.03)
        self.declare_parameter('min_distance', 8)
        self.declare_parameter('win_size', 15)
        self.declare_parameter('max_level', 2)
        self.declare_parameter('scale_factor', 0.2)

        self.image_topic = self.get_parameter('image_topic').value
        self.caminfo_topic = self.get_parameter('camera_info_topic').value
        self.max_corners = self.get_parameter('max_corners').value
        self.quality_level = self.get_parameter('quality_level').value
        self.min_distance = self.get_parameter('min_distance').value
        self.win_size = self.get_parameter('win_size').value
        self.max_level = self.get_parameter('max_level').value
        self.scale_factor = self.get_parameter('scale_factor').value

        self.bridge = CvBridge()
        self.K = None
        self.cam_received = False

        self.prev_gray = None
        self.prev_pts = None

        self.R = np.eye(3)
        self.t = np.zeros(3)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vodom_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(CameraInfo, self.caminfo_topic, self.caminfo_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        self.get_logger().info('Mono VO node started')

    def caminfo_cb(self, msg: CameraInfo):
        if not self.cam_received:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3,3))
            self.cam_received = True
            self.get_logger().info(f'CameraInfo received, K=\n{self.K}')

    def detect_features(self, gray):
        corners = cv2.goodFeaturesToTrack(
            gray, maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            blockSize=7
        )
        if corners is None:
            return None
        return corners.reshape(-1,2)

    def image_cb(self, msg: Image):
        if not self.cam_received:
            self.get_logger().warning_once('Waiting for CameraInfo...')
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < 20:
            pts = self.detect_features(gray)
            if pts is None:
                return
            self.prev_pts = pts.reshape(-1,1,2).astype(np.float32)
            self.prev_gray = gray
            self.get_logger().info(f'Initialized {len(self.prev_pts)} features')
            return

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(self.win_size,self.win_size),
            maxLevel=self.max_level,
            criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01)
        )

        if next_pts is None:
            self.prev_pts = None
            self.prev_gray = gray
            return

        status = status.reshape(-1)
        prev_good = self.prev_pts.reshape(-1,2)[status==1]
        next_good = next_pts.reshape(-1,2)[status==1]

        if len(prev_good) < 8:
            self.prev_pts = None
            self.prev_gray = gray
            return

        E, mask = cv2.findEssentialMat(next_good, prev_good, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            self.prev_pts = None
            self.prev_gray = gray
            return

        _, R_est, t_est, mask_pose = cv2.recoverPose(E, next_good, prev_good, self.K)
        t_vec = t_est.reshape(3) * self.scale_factor

        t_world = self.R.dot(t_vec)
        self.t += t_world
        self.R = self.R.dot(R_est)

        # Convert OpenCV camera frame to ROS base_link frame
        t_ros = np.array([t_world[2], -t_world[0], -t_world[1]])

        self.t_ros = getattr(self, 't_ros', np.zeros(3)) + t_ros

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.t_ros[0])
        odom.pose.pose.position.y = float(self.t_ros[1])
        odom.pose.pose.position.z = float(self.t_ros[2])
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        tmsg = TransformStamped()
        tmsg.header.stamp = odom.header.stamp
        tmsg.header.frame_id = 'odom'
        tmsg.child_frame_id = 'base_link'
        tmsg.transform.translation.x = float(self.t_ros[0])
        tmsg.transform.translation.y = float(self.t_ros[1])
        tmsg.transform.translation.z = float(self.t_ros[2])
        tmsg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tmsg)

        pose = PoseStamped()
        pose.header = odom.header
        pose.header.frame_id = 'map'
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        if len(next_good) < 50:
            pts = self.detect_features(gray)
            if pts is not None:
                self.prev_pts = pts.reshape(-1,1,2).astype(np.float32)
            else:
                self.prev_pts = next_good.reshape(-1,1,2).astype(np.float32)
        else:
            self.prev_pts = next_good.reshape(-1,1,2).astype(np.float32)
        self.prev_gray = gray


def main(args=None):
    rclpy.init(args=args)
    node = MonoVONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
