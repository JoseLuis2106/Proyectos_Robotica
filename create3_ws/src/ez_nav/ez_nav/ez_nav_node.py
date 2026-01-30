#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import Undock, RotateAngle, Dock

import numpy as np
import time
import signal
import math

class Navigation(Node):
    def __init__(self):
        """
        Nodo de navegación autónoma de iRobot Create3.
        """
        super().__init__('navigation')

        qos_profile = QoSProfile(
            depth = 10,
            reliability = ReliabilityPolicy.BEST_EFFORT
        )

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscripciones
        # Topics de iRobot Create3
        self.ir_sub = self.create_subscription(
            IrIntensityVector, '/ir_intensity', self.ir_cb, qos_profile
        )
        self.pose_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, qos_profile
        )
        
        # Topic para finalizar la ejecución
        self.stop_sub = self.create_subscription(
            Bool, '/stop_navigation', self.stop_cb, 10
        )

        # Action clients iRobot Create3
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')
        self.dock_client = ActionClient(self, Dock, '/dock')

        # Parámetros
        self.linear_speed = 0.1
        self.obs_threshold = 35
        self.side_threshold = 20

        # Sensores IR
        self.front_ir = 0
        self.left_ir = 0
        self.right_ir = 0

        # Estados
        self.state = 0
        self.des_dir = 0                        # 0: derecha, 1: izquierda
        self.turning = False

        # Odometría
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.init = True
        self.init_x = 0.0
        self.init_y = 0.0
        self.n_metros = 1.5                     # Distancia en metros entre giros (sin contar giros por obstáculos)

        # Gestion señal SIGINT
        signal.signal(signal.SIGINT, self.sigint_handler)

        self.get_logger().info("Nodo iniciado – saliendo del dock...")
        self.send_undock()

        self.stop_cond = False                  # Condición de parada

        time.sleep(1)


    def sigint_handler(self, sig, frame): 
        """
        Al recibir SIGINT, se envía la orden de volver al dock y activa condición de parada.
        """

        self.get_logger().info("Terminando nodo por SIGINT")
        self.send_dock()
        self.stop_cond = True


    def stop_cb(self, msg):                     
        """
        Al recibir la orden de parar por el servidor web, se envía la orden de volver al dock y activa condición de parada.
        """
        
        if msg.data == True:
            self.get_logger().info("Terminando nodo por servidor")
            self.send_dock()
            self.stop_cond = True


    def odom_cb(self, msg):
        """
        Actualiza la información de la pose actual del robot.
        """

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        or_q = msg.pose.pose.orientation
        _, _, self.th = euler_from_quaternion([or_q.x, or_q.y, or_q.z, or_q.w])

        if self.init:                           # Reinicio de "posición inicial" (reinicia distancia recorrida para comparar con n_metros)
            self.init = False
            self.init_x = self.x
            self.init_y = self.y
            self.init_z = self.th
            self.n_metros = 1.5

    def distance(self):
        """
        Devuelve la distancia recorrida por el robot desde el último reinicio.
        """

        return np.sqrt((self.x-self.init_x)**2 + (self.y-self.init_y)**2)
    
    def send_undock(self):
        """
        Envía la orden al robot de abandonar el dock.
        """

        self.undock_client.wait_for_server()
        self.undock_client.send_goal_async(Undock.Goal())

    def rotate_90(self, direction):
        """
        Envía al robot la orden de girar 90º en una determinada dirección.
        Dirección: 0 -> Derecha.
        Dirección: 1 -> Izquierda.
        """

        if self.turning:
            return

        self.turning = True
        angle = math.pi / 2
        if direction == 0:                      # Si giro a la derecha, angulo opuesto
            angle = -angle

        goal = RotateAngle.Goal()
        goal.angle = angle
        goal.max_rotation_speed = 1.0

        self.rotate_client.wait_for_server()
        future = self.rotate_client.send_goal_async(goal)
        future.add_done_callback(self.rotation_done)
        self.des_dir = 1 - direction            # Se actualiza la próxima dirección deseada

    def rotation_done(self):
        """
        Actualización de parámetros tras girar (fin de giro, vuelta a estado 0 y reinicio de posición inicial).
        """

        self.turning = False
        self.state = 0
        self.init = True                        # Reinicio de "posición inicial" (reinicia distancia recorrida para comparar con n_metros)

    def send_dock(self):
        """
        Envía la orden al robot de regresar al dock.
        """

        self.get_logger().info("Volviendo al dock...")
        self.dock_client.wait_for_server()
        self.dock_client.send_goal_async(Dock.Goal())


    def ir_cb(self, msg):
        """
        Recibe las lecturas de los sensores IR del robot.

        Se usa además para actualizar el estado del robot y decidir su próximo movimiento.
        El robot se desplazará en línea recta, alternando giros de 90º en varias direcciones para abarcar la mayor superficie posible.
        El robot alternará, si es posible, giros a la izquierda y a la derecha. Se entiende por 'dirección deseada' la opuesta al último giro que se hizo.
        La gestión del movimiento se lleva a cabo mediante una máquina de estados:
        0: Desplazamiento hacia adelante si es posible.
        1: Intento de giro en la dirección deseada.
        2: Reacción del robot a un obstáculo delante.
        3: Giro del robot en la dirección no deseada tras avanzar demasiados metros sin poder girar en la dirección deseada.
        """

        front_values = []
        left_values = []
        right_values = []

        # Lectura de los sensores IR
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


        # Gestión del estado del robot y su movimiento
        # Se mueve recto hasta obstaculo o cambio de direccion (n_metros)
        if self.state == 0:
            if self.front_ir > self.obs_threshold:
                self.state = 2                              # Obstáculo delante

            elif self.distance() >= self.n_metros and (self.des_dir == 0 and self.right_ir < self.side_threshold or self.des_dir == 1 and self.left_ir < self.side_threshold):
                self.state = 1                              # Intento de giro en la dirección deseada

            elif self.distance() >= 3*self.n_metros:
                self.state = 3                              # Intento de giro en la dirección no desada (evitar recorrer muchos metros en la misma dirección)

            else:
                self.twist.linear.x = self.linear_speed     # Desplazamiento hacia adelante


        # Giro deseado si es posible
        if self.state == 1:
            if (self.des_dir == 0 and self.right_ir < self.side_threshold or self.des_dir == 1 and self.left_ir < self.side_threshold):
                self.rotate_90(self.des_dir)

            else:
                self.state = 0

        # Obstáculo delante
        if self.state == 2:
            if self.front_ir < self.obs_threshold:          # Ya no hay obstaculo delante tras girar
                self.state = 0

            elif (self.des_dir == 0 and self.right_ir < self.side_threshold or self.des_dir == 1 and self.left_ir < self.side_threshold):
                self.rotate_90(self.des_dir)                # Evita el obstáculo en la dirección deseada si es posible

            elif (self.des_dir == 0 and self.right_ir > self.side_threshold or self.des_dir == 1 and self.left_ir > self.side_threshold):
                self.rotate_90(1-self.des_dir)              # Evita el obstáculo en la dirección no deseada si es posible


        # Giro no deseado tras avanzar muchos metros sin ser posible el giro
        if self.state == 3:
            if self.des_dir == 1 and self.right_ir < self.side_threshold or self.des_dir == 0 and self.left_ir < self.side_threshold:
                self.rotate_90(1-self.des_dir)              # Giro no deseado si es posible

            else:
                self.state = 0


        self.cmd_pub.publish(self.twist)



def main(args=None):
    """
    Nodo de navegación activo siempre que no se active la condición de parada. 
    La condición de parada se da si se recibe una señal SIGINT (Ctrl+C) o si se pulsa el botón de parada desde la aplicación web.
    """
    rclpy.init(args=args)
    node = Navigation()
    while rclpy.ok() and not node.stop_cond:
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
