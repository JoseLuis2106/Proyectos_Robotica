from flask import Flask, render_template, redirect, url_for, request
import os

app = Flask(__name__)
UPLOAD_FOLDER = "static/uploads"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

robot_running = False
last_image = None  # Para mostrar la última imagen subida
people_detected = 0

@app.route("/")
def index():
    print(render_template(
        "index.html",
        robot_running=robot_running,
        last_image=last_image,
        people=people_detected  # <--- agregamos esta línea
    ))
    return render_template(
        "index.html",
        robot_running=robot_running,
        last_image=last_image,
        people=people_detected
    )
@app.route("/start")
def start_robot():
    global robot_running
    robot_running = True
    print("Robot iniciado")
    return redirect(url_for("index"))

@app.route("/stop")
def stop_robot():
    global robot_running
    robot_running = False
    print("Robot detenido")
    return redirect(url_for("index"))

# 🔹 ENDPOINT PARA RECIBIR IMAGEN + NUMERO DE PERSONAS
@app.route("/upload-image/", methods=["POST"])
def upload_image():
    global last_image, people_detected
    
    # Validar imagen
    if "file" not in request.files:
        return "No se envió la imagen", 400

    file = request.files["file"]
    if not file.mimetype.startswith("image/"):
        return "El archivo no es una imagen", 400

    # Validar número de personas
    if "people" not in request.form:
        return "No se envió la cantidad de personas", 400

    try:
        people_detected = int(request.form["people"])
    except ValueError:
        return "La cantidad de personas debe ser un número", 400

    # Guardar imagen
    filename = f"uploaded_{file.filename}"
    filepath = os.path.join(UPLOAD_FOLDER, filename)
    file.save(filepath)

    print(f" filepath: {filepath}")
    last_image = f"uploads/{filename}"  # ruta relativa para url_for

    print(f"Imagen recibida: {filename} | Personas: {people_detected}")
    # 🔹 Redirigir al index para refrescar el HTML con los datos actualizados
    return redirect(url_for("index"))

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=True)
