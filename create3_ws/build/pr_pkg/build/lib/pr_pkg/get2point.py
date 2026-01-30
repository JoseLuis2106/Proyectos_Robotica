import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time

class Get2Point(Node):
    def __init__(self):
        super().__init__('Get2Point_n')

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, qos)

        self.init = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.goal_x = 2.0
        self.goal_y = 0.0
        self.goal_tol = 0.2

        self.k_lin = 0.5
        self.k_ang = 1.5

        time.sleep(5)

        self.timer = self.create_timer(0.1, self.ctrl)
        self.get_logger().info("Nodo de control iniciado.")

    def odom_cb(self, msg):
        #self.get_logger().info("Odom recibido")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        or_q = msg.pose.pose.orientation
        _, _, self.th = euler_from_quaternion([or_q.x, or_q.y, or_q.z, or_q.w])

        

        if not self.init:
            self.init = 1
            self.goal_x = 0.0 + self.x
            self.goal_y = 2.0 + self.y
            self.goal_tol = 0.2


    def ctrl(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        dist = math.sqrt(dx**2 + dy**2)
        ang_act = math.atan2(dy, dx)
        ang_dist = self.norm(ang_act - self.th)

        self.get_logger().info(f"Distancia actual al goal: {dist:.2f}")

        twist = Twist()

        if dist > self.goal_tol:
            twist.linear.x = self.k_lin * dist
            twist.angular.z = self.k_ang * ang_dist
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Punto alcanzado.")

        self.cmd_pub.publish(twist)

    @staticmethod
    def norm(ang):
        while ang > math.pi:
            ang -= 2 * math.pi
        while ang < -math.pi:
            ang += 2 * math.pi
        return ang

def main(args=None):
    rclpy.init(args=args)
    node = Get2Point()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == "__main__":
    main()
