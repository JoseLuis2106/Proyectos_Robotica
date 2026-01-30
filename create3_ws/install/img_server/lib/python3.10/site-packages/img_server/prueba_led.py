#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import LightringLeds, LedColor

class LightringNode(Node):
    def __init__(self):
        super().__init__('lightring_node')

        # Publicador del aro de luz
        self.lightring_pub = self.create_publisher(LightringLeds, '/cmd_lightring', 10)

        # Poner aro verde al inicio
        self.set_ring_color(0, 255, 0)
        self.get_logger().info("Aro inicializado en verde")

    def set_ring_color(self, red: int, green: int, blue: int):
        msg = LightringLeds()
        msg.override_system = True
        # Crear 6 leds con el mismo color
        msg.leds = [LedColor(red=red, green=green, blue=blue) for _ in range(6)]
        self.lightring_pub.publish(msg)
        self.get_logger().info(f"Aro puesto en color R:{red} G:{green} B:{blue}")

def main(args=None):
    rclpy.init(args=args)
    node = LightringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
