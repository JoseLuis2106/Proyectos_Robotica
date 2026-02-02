import cv2
from picamera2 import Picamera2
import libcamera
import numpy as np
from datetime import datetime
import os
import requests
import time

TIEMPO_ESPERA = 2   # Tiempo de espera en segundos hasta transición de estado 1 a 0
ESTADO_0 = 0        # Sin detección de personas
ESTADO_1 = 1        # Se detectan personas


# ----------- CONFIGURACIÓN MODELO DNN -----------
prototxt = "MobileNetSSD_deploy.prototxt.txt"
model = "MobileNetSSD_deploy.caffemodel"

classes = {0: "background", 1: "aeroplane", 2: "bicycle", 3: "bird", 4: "boat",
           5: "bottle", 6: "bus", 7: "car", 8: "cat", 9: "chair", 10: "cow",
           11: "diningtable", 12: "dog", 13: "horse", 14: "motorbike",
           15: "person", 16: "pottedplant", 17: "sheep", 18: "sofa",
           19: "train", 20: "tvmonitor"}

net = cv2.dnn.readNetFromCaffe(prototxt, model)


# ----------- CONFIGURACIÓN DE LA CÁMARA -----------
camera = Picamera2()
camera_config = camera.create_preview_configuration(
    main={"size": (640, 480)},
    display="main"
)
camera_config['transform'] = libcamera.Transform(vflip=True)
camera.configure(camera_config)
camera.start()
print("✅ Cámara iniciada. Presiona 'q' para salir.")

# ----------- CONFIGURACIÓN HTTP -----------
SERVER_URL = "http://10.105.3.45:8000/upload-image"         # Servidor web local
LOCAL_SAVE_DIR = "/home/miera/desktop/imgs/"                # Directorio de almacenamiento de imágenes capturadas
os.makedirs(LOCAL_SAVE_DIR, exist_ok=True)


class SistemaPersonas:
    """
    Sistema de envío de imágenes a servidor web basado en una máquina de estados.
    """

    def __init__(self):
        self.estado = ESTADO_0
        self.contador_personas = 0
        self.tiempo_inicio = 0.0
        self.contando = False
        

    def enviar_foto(self, foto, frame, personas_detectadas):
        """
        Lógica para el envío de imágenes al servidor FastAPI.
        Envío de imagen y número de personas detectadas vía HTTP y eliminación de su copia local.
        """

        timestamp = datetime.now().strftime("%y%m%d%H%M%S") + str(int(datetime.now().microsecond / 100000))
        output_path = os.path.join(LOCAL_SAVE_DIR, f"{timestamp}.png")
        cv2.imwrite(output_path, frame)
        print(f"📸 Persona detectada! Imagen guardada: {output_path}")

        try:
            with open(output_path, "rb") as f:
                files = {"file": (os.path.basename(output_path), f, "image/png")}
                print(f"personas_detectadas: {personas_detectadas}")
                r = requests.post(SERVER_URL, files=files, data = {     # Envío de imagen y del número de personas detectadas
                        "people": personas_detectadas
                    })
                
            if r.status_code == 200:
                print("📤 Imagen enviada correctamente al servidor FastAPI.")
                os.remove(output_path)                                  # Eliminación de la copia local de la imagen
                print("🗑️ Imagen eliminada del disco local.")
            else:
                print("❌ Error al enviar imagen, status code:", r.status_code)

        except Exception as e:                                          # Tratamiento de errores para evitar el bloqueo
            print("❌ Error enviando la imagen:", e)


    def iniciar_contador(self, tiempo_actual):
        """
        Se inicializa el contador siempre que se detecte alguna persona en la imagen.
        Para regresar al estado 0 debe transcurrir TIEMPO_ESPERA segundos desde la última detección.
        """

        self.tiempo_inicio = tiempo_actual
        self.contando = True
        print("Iniciando contador de tiempo...")
    

    def actualizar(self,personas_detectadas,tiempo_actual,frame): 
        """
        Actualización de la máquina de estados y envío de la imagen en función del número de detecciones y del tiempo transcurrido.
        """

        if self.estado == ESTADO_0:         # Estado actual: sin personas en la imagen
            if personas_detectadas > 0:
                self.contador_personas = personas_detectadas
                self.enviar_foto("foto_persona.jpg",frame,personas_detectadas)                  # Envío de la imagen
                self.iniciar_contador(tiempo_actual)
                self.estado = ESTADO_1

        elif self.estado == ESTADO_1:       # Estado actual: presencia de personas en la imagen
            if personas_detectadas > 0:
                self.iniciar_contador(tiempo_actual)                                            # Reinicia contador si hay alguna persona

            if personas_detectadas > self.contador_personas:                                    # Aumenta el número de detecciones
                self.contador_personas = personas_detectadas
                self.enviar_foto("foto_persona.jpg",frame,personas_detectadas)                  # Envío de la imagen

            elif personas_detectadas == 0:                                                      # Sin detecciones
                if self.contando and (tiempo_actual - self.tiempo_inicio) >= TIEMPO_ESPERA:     # Vuelta a estado 0 tras 2 segundos sin detecicones
                    self.estado = ESTADO_0
                    self.contador_personas = 0
                    self.contando = False
                    print("No se detectan personas. Volviendo al Estado 0.")


def main(args=None):
    """
    Se capturan imágenes continuamente y se detectan personas mediante el modelo DNN.
    Envío de imagen y número de detecciones al servidor web según máquina de estados.
    """

    sistema = SistemaPersonas()
    tiempo_actual = time.time()  # Para medir el tiempo transcurrido sin detecciones.
    while True:
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        height, width, _ = frame.shape
        
        # Preparar imagen para la DNN y detecciones
        image_resized = cv2.resize(frame, (300, 300))
        blob = cv2.dnn.blobFromImage(image_resized, 0.007843, (300, 300),
                                 (127.5, 127.5, 127.5), swapRB=False, crop=False)
        net.setInput(blob)
        detections = net.forward()
        
        # Estimación del número de personas a partir de la salida del modelo y dibujo de bounding boxes
        person_boxes = []
        for detection in detections[0][0]:
            confidence = float(detection[2])
            class_id = int(detection[1])
            if confidence > 0.45 and classes.get(class_id) == "person":         # Obtención de bounding boxes de las personas detectadas y nivel de confianza.
                box = detection[3:7] * np.array([width, height, width, height])
                box = box.astype("int")
                person_boxes.append(box)

                # Dibujo de bounding box
                x_start, y_start, x_end, y_end = box
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)

                # Confianza de la detección
                label = f"Person {confidence*100:.1f}%"
                cv2.putText(frame, label, (x_start, max(20, y_start - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        current_person_count = len(person_boxes)                                # Número de detecciones de personas en la imagen
        
        tiempo_actual = time.time()
        sistema.actualizar(current_person_count, tiempo_actual, frame)          # Actualización del sistema

        time.sleep(0.03)
        
        
if __name__ == "__main__":
    main()