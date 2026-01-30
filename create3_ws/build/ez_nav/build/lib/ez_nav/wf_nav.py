#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector
from irobot_create_msgs.action import Undock, RotateAngle, Dock

import numpy as np
import time
import signal
import math
import os

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscripciones
        self.ir_sub = self.create_subscription(
            IrIntensityVector, '/ir_intensity', self.ir_cb, qos_profile
        )
        self.pose_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, qos_profile
        )
        self.stop_sub = self.create_subscription(
            Bool, '/stop_navigation', self.stop_cb, 10
        )

        # Action clients
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')
        self.dock_client = ActionClient(self, Dock, '/dock')

        # Parámetros
        self.linear_speed = 0.1
        self.obs_threshold = 35
        self.side_threshold = 20

        # Sensores
        self.front_ir = 0
        self.left_ir = 0
        self.right_ir = 0

        # Estados
        self.state = 0
        self.des_dir = 0  # 0: derecha, 1: izquierda
        self.turning = False

        # Odometría
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.init = True
        self.init_x = 0.0
        self.init_y = 0.0
        self.n_metros = 1.5

        # Señal Ctrl+C
        signal.signal(signal.SIGINT, self.sigint_handler)

        self.get_logger().info("Nodo iniciado – saliendo del dock...")
        self.send_undock()

        self.bump = False
        self.stop_cond = False

        time.sleep(1)

    def hazard(self, msg):
        self.get_logger().info("Peligro detectado")
        self.bump = True

    def sigint_handler(self, sig, frame):
        self.get_logger().info("Terminando nodo por SIGINT")
        self.send_dock()
        self.stop_cond = True

    def stop_cb(self, msg):
        if msg.data == True:
            self.get_logger().info("Terminando nodo por servidor")
            self.send_dock()
            self.stop_cond = True

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        or_q = msg.pose.pose.orientation
        _, _, self.th = euler_from_quaternion([or_q.x, or_q.y, or_q.z, or_q.w])

        if self.init:
            # self.get_logger().info(f"Reinicio pose inicial")
            self.init = False
            self.init_x = self.x
            self.init_y = self.y
            self.init_z = self.th
            self.n_metros = 1.5

    def distance(self):
        return np.sqrt((self.x-self.init_x)**2 + (self.x-self.init_x)**2)
    
    def send_undock(self):
        self.undock_client.wait_for_server()
        self.undock_client.send_goal_async(Undock.Goal())

    def rotate_90(self, direction):
        if self.turning:
            return

        self.turning = True
        angle = math.pi / 2
        if direction == 0:  # derecha
            angle = -angle

        goal = RotateAngle.Goal()
        goal.angle = angle
        goal.max_rotation_speed = 1.0

        self.rotate_client.wait_for_server()
        future = self.rotate_client.send_goal_async(goal)
        future.add_done_callback(self.rotation_done)
        self.des_dir = 1 - direction
        # self.get_logger().info(f"Giro hecho: {direction}")

    def rotation_done(self, future):
        self.turning = False
        self.state = 0
        self.init = True

    def send_dock(self):
        self.get_logger().info("Volviendo al dock...")
        self.dock_client.wait_for_server()
        self.dock_client.send_goal_async(Dock.Goal())


    def ir_cb(self, msg):
        front_values = []
        left_values = []
        right_values = []

        for reading in msg.readings:
            name = reading.header.frame_id
            value = reading.value

            if 'front' in name:
                front_values.append(value)
            elif 'left' in name:
                left_values.append(value)
            elif 'right' in name:
                right_values.append(value)

        self.front_ir = max(front_values) if front_values else 0
        self.left_ir = max(left_values) if left_values else 0
        self.right_ir = max(right_values) if right_values else 0

        self.twist = Twist()

        # if self.front_ir > self.obs_threshold or self.left_ir > self.obs_threshold or self.right_ir > self.obs_threshold:
        #     if self.left_ir < self.right_ir:
        #         twist.angular.z = self.angular_speed
        #         self.get_logger().info("Obstacle ahead! Turning LEFT")
        #     else:
        #         twist.angular.z = -self.angular_speed
        #         self.get_logger().info("Obstacle ahead! Turning RIGHT")
        #     twist.linear.x = 0.0
        # else:
        #     # Path clear
        #     twist.linear.x = self.linear_speed
        #     twist.angular.z = 0.0


        if self.state == 0:                             # Se mueve recto hasta obstaculo o cambio de direccion (n metros)
            if self.front_ir > self.obs_threshold or self.bump == True:
                # self.get_logger().info(f"Estado 0, obstaculo delante, cambio a estado 2")
                self.state = 2

            elif self.distance() >= self.n_metros and (self.des_dir == 0 and self.right_ir < self.side_threshold or self.des_dir == 1 and self.left_ir < self.side_threshold):
                # self.get_logger().info(f"Estado 0, intento de giro deseado (estado 1)")
                self.state = 1

            elif self.distance() >= 3*self.n_metros:
                # self.get_logger().info(f"Estado 0, muchos metros sin direccion deseada posible, giro direccion no deseada (estado 3)")
                self.state = 3

            else:
                # self.get_logger().info(f"Estado 0, yendo adelante {self.distance()}/{self.n_metros}")
                self.twist.linear.x = self.linear_speed

            
        if self.state == 1:                             # Giro deseado si posible
            if (self.des_dir == 0 and self.right_ir < self.side_threshold or self.des_dir == 1 and self.left_ir < self.side_threshold):
                # self.get_logger().info(f"Estado 1, girando en direccion deseada ({self.des_dir})")
                self.rotate_90(self.des_dir)
                # self.cambiar_dir = 1
                # self.get_logger().info(f"Giro deseado: {self.des_dir}")

            else:
                # self.get_logger().info(f"Estado 1, vuelta a estado 0")
                self.state = 0
                # self.des_dir = 1-self.des_dir if self.cambiar_dir else self.des_dir
                # self.cambiar_dir = 0
                self.cont = 0


        if self.state == 2:                             # Obstaculo delante
            self.bump = False

            if self.front_ir < self.obs_threshold:     #Termina de girar y no hay obstaculo delante
                # self.get_logger().info(f"Estado 2, vuelta a estado 0")
                self.state = 0
                # self.des_dir = 1-self.des_dir if self.cambiar_dir else self.des_dir
                # self.cambiar_dir = 0

            elif (self.des_dir == 0 and self.right_ir < self.side_threshold or self.des_dir == 1 and self.left_ir < self.side_threshold):
                # self.get_logger().info(f"Estado 2, esquivando obstaculo en direccion deseada")
                self.rotate_90(self.des_dir)
                # self.cambiar_dir = 1
                # self.get_logger().info(f"Giro deseado: {self.des_dir}")

            elif (self.des_dir == 0 and self.right_ir > self.side_threshold or self.des_dir == 1 and self.left_ir > self.side_threshold):
                # self.get_logger().info(f"Estado 2, esquivando obstaculo en direccion no deseada")
                self.rotate_90(1-self.des_dir)
                # self.get_logger().info(f"Giro deseado: {self.des_dir}")

            # else:
            #     if self.right_ir < self.left_ir:            # Obstaculo mas cercano a la izq
            #         self.rotate_90(0)                       # Giro a la derecha
            #         # self.get_logger().info(f"Ultimo giro a la derecha para evitar obstaculo")
            #     else:                                       # Obstaculo mas cercano a la izq
            #         self.rotate_90(1)                       # Giro a la izquierda
            #         # self.get_logger().info(f"Ultimo giro a la izquierda para evitar obstaculo")


        if self.state == 3:                             # Giro no deseado
            if self.des_dir == 1 and self.right_ir < self.side_threshold or self.des_dir == 0 and self.left_ir < self.side_threshold:
                # self.get_logger().info(f"Estado 3, girando en direccion no deseada")
                self.rotate_90(1-self.des_dir)
                # self.get_logger().info(f"Giro deseado: {self.des_dir}")

            else:
                # self.get_logger().info(f"Estado 3, vuelta a estado 0")
                self.state = 0



        #twist.angular.z = 0.0
        #twist.linear.x = 0.0
        self.cmd_pub.publish(self.twist)
        #self.get_logger().info(f"Front IR: {self.front_ir}, Left IR: {self.left_ir}, Right IR: {self.right_ir}")

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    while rclpy.ok() and not node.stop_cond:
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
