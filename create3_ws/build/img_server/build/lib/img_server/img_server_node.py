#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from irobot_create_msgs.msg import LightringLeds, LedColor

import threading
import os
import time

from flask import Flask, request, redirect, url_for, render_template

import importlib.resources as pkg_resources
import img_server

class ImgServerNode(Node):

    def __init__(self):
        super().__init__('img_server_node')
        """
        Nodo servidor para recepción de imágenes enviadas por Raspberry Pi y para monitorización de la vigilancia mediante aplicación web.
        """

        # Parámetros del servidor
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('timeout', 10.0)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.timeout = self.get_parameter('timeout').value


        # Inicialización del color del anillo de iRobot Create3
        self.lightring_pub = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            10
        )

        self.current_color = (0, 200, 0)            # Verde
        self.last_image_time = time.time()          # Tiempo de referencia para la actualización del color del anillo

        self.set_ring_color(*self.current_color)
        self.create_timer(0.5, self.check_timeout)



        # Aplicación Flask
        templates_path = '/home/jlmre/create3_ws/src/img_server/templates'
        static_path = '/home/jlmre/create3_ws/src/img_server/static'

        self.app = Flask(
            __name__,
            template_folder=str(templates_path),
            static_folder=str(static_path),
        )

        self.last_image = None
        self.people_detected = 0
        self.save_dir = static_path


        # Gestión de la terminación de los nodos por la aplicación web
        self.stop_cond = False

        self.stop_pub = self.create_publisher(
            Bool,
            '/stop_navigation',
            10
        )
        

        @self.app.route("/stop")
        def stop_robot():
            """
            Recepción de la solicitud de parada por la aplicación web.
            """

            self.get_logger().warn("STOP solicitado desde interfaz web")

            msg = Bool()
            msg.data = True
            self.stop_pub.publish(msg)

            self.stop_cond = True

            return render_template(                     # Renderizado de la aplicación web
                "index.html",
                robot_running=False,
                last_image=self.last_image,
                people=self.people_detected
            )


        
        @self.app.route("/")
        def index():
            """
            Renderizado de la aplicación web.
            """

            return render_template(
                "index.html",
                robot_running=True,
                last_image=self.last_image,
                people=self.people_detected
            )


        @self.app.route("/upload-image/", methods=["POST"])
        def upload_image():
            """
            Gestión de la recepción de imágenes.
            """

            if "file" not in request.files:
                return "No se envió la imagen", 400

            file = request.files["file"]
            
            data = request.form.get("people")
            print(f"Dato: {data}")

            if not file.mimetype.startswith("image/"):
                return "El archivo no es una imagen", 400

            # Borrar imágenes anteriores
            for fname in os.listdir(self.save_dir):
                fpath = os.path.join(self.save_dir, fname)
                if os.path.isfile(fpath) and not fname.lower().endswith(".css"):
                    os.remove(fpath)

            # Guardar última imagen
            filename = f"{file.filename}"
            filepath = os.path.join(self.save_dir,filename)
            file.save(filepath)

            # Actualización de "última imagen recibida" (para su publicación en la aplicación web)
            self.last_image = f"{file.filename}"
            self.people_detected = max(int(data), self.people_detected)

            self.get_logger().info(f"Image received: {filepath}")
            self.get_logger().warn(f"Persona detectada {time.asctime()}")

            # Cambiar aro a naranja
            self.set_ring_color(200, 60, 0)
            self.last_image_time = time.time()

            return redirect(url_for("index"))


        # Ejecución del hilo Flask
        self.get_logger().info(
            f"Starting image server on http://{self.host}:{self.port}"
        )

        self.server_thread = threading.Thread(
            target=self.run_server,
            daemon=True
        )
        self.server_thread.start()


    def run_server(self):
        """
        Ejecución del servidor.
        """
        self.app.run(
            host=self.host,
            port=self.port,
            debug=False,
            use_reloader=False
        )

    # -------------------------
    # ROS: GESTIÓN LIGHTRING
    # -------------------------
    def set_ring_color(self, red: int, green: int, blue: int):
        """
        Gestión del color del anillo de iRobot Create 3. Almecena el color actual para evitar enviar repetidamente el mismo mensaje al topic /cmd_lightring.
        """

        if self.current_color != (red, green, blue):
            msg = LightringLeds()
            msg.override_system = True
            msg.leds = [
                LedColor(red=red, green=green, blue=blue)
                for _ in range(6)
            ]
            self.lightring_pub.publish(msg)
            self.get_logger().info(
                f"Aro puesto en color R:{red} G:{green} B:{blue}"
            )
            self.current_color = (red, green, blue)

    def check_timeout(self):
        """
        Comprobación del timer para que si el tiempo transcurrido es mayor que self.timeout se reinicie el color a verde.
        """

        if time.time() - self.last_image_time > self.timeout:
            self.set_ring_color(0, 200, 0)
            self.last_image_time = time.time()




def main(args=None):
    """
    Nodo de navegación activo siempre que no se active la condición de parada. 
    La condición de parada se da si se pulsa el botón de parada desde la aplicación web.
    """
    rclpy.init(args=args)
    node = ImgServerNode()
    while rclpy.ok() and not node.stop_cond:
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
