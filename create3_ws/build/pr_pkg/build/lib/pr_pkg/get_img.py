import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time

class GetImg(Node):
    def __init__(self):
        super().__init__('GetImg_n')
        
        #qos = QoSProfile(depth=10)
        #qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(Odometry, '/camera/image', self.image_cb)


        time.sleep(5)

        self.timer = self.create_timer(0.1, self.ctrl)
        self.get_logger().info("Nodo de recepcion de imagenes iniciado.")

    def image_cb(self, msg):
        #self.get_logger().info("Odom recibido")
        pass
        


    

def main(args=None):
    rclpy.init(args=args)
    node = GetImg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == "__main__":
    main()
