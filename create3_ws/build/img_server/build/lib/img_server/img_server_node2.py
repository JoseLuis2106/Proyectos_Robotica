#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import os
from flask import Flask, render_template, redirect, url_for, request
import time

from fastapi import FastAPI, File, UploadFile
from fastapi.responses import JSONResponse
import uvicorn

from irobot_create_msgs.msg import LightringLeds, LedColor


class ImgServerNode(Node):

    def __init__(self):
        super().__init__('img_server_node')

        # Parámetros ROS
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('save_dir', 'tmp/img_server')
        self.declare_parameter('timeout', 10.0)  # tiempo sin imágenes para volver a verde

        self.lightring_pub = self.create_publisher(LightringLeds, '/cmd_lightring', 10)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.save_dir = self.get_parameter('save_dir').value
        self.timeout = self.get_parameter('timeout').value

        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(f"Starting image server on http://{self.host}:{self.port}")

        # Última vez que se recibió una imagen
        self.last_image_time = time.time()
        # Color actual del aro
        self.current_color = (0, 200, 0)  # verde inicial

        # Publicar verde al iniciar
        self.set_ring_color(*self.current_color)

        # Timer ROS para revisar timeout
        self.create_timer(0.5, self.check_timeout)

        # Crear app FastAPI
        self.app = FastAPI()

        @self.app.post("/upload-image/")
        async def upload_image(file: UploadFile = File(...)):
            if not file.content_type.startswith("image/"):
                return JSONResponse({"error": "File is not an image"}, status_code=400)
            
            for fname in os.listdir(self.save_dir):
                fpath = os.path.join(self.save_dir, fname)
                if os.path.isfile(fpath):
                    os.remove(fpath)

            image_bytes = await file.read()
            filepath = os.path.join(self.save_dir, file.filename)
            with open(filepath, "wb") as f:
                f.write(image_bytes)

            self.get_logger().info(f"Image received: {filepath}")
            self.get_logger().warn(f"Persona detectada {time.asctime()}")

            # Cambiar aro a naranja al recibir imagen
            self.set_ring_color(200, 60, 0)

            # Actualizar el tiempo del último mensaje
            self.last_image_time = time.time()

            return {
                "filename": file.filename,
                "content_type": file.content_type,
                "size_bytes": len(image_bytes)
            }

        # Lanzar FastAPI en hilo separado
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

    def run_server(self):
        uvicorn.run(self.app, host=self.host, port=self.port, log_level="info")

    def set_ring_color(self, red: int, green: int, blue: int):
        # Solo publicar si cambia el color
        if self.current_color != (red, green, blue):
            msg = LightringLeds()
            msg.override_system = True
            msg.leds = [LedColor(red=red, green=green, blue=blue) for _ in range(6)]
            self.lightring_pub.publish(msg)
            self.get_logger().info(f"Aro puesto en color R:{red} G:{green} B:{blue}")
            self.current_color = (red, green, blue)

    def check_timeout(self):
        if time.time() - self.last_image_time > self.timeout:
            # Volver a verde si han pasado más de timeout segundos
            self.set_ring_color(0, 200, 0)
            self.last_image_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = ImgServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
