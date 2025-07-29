import cv2
import numpy as np

# Parámetros físicos
area_real_m = 1.22         # ⚠️ Nueva área real deseada (1.22 m x 1.22 m)
altura_camara_m = 1.38     # Altura de la cámara desde el plano de trabajo
fov_horizontal_deg = 90    # Ángulo de visión horizontal en grados

# Calcular el ancho real que ve la cámara usando trigonometría
fov_horizontal_rad = np.deg2rad(fov_horizontal_deg)
fov_ancho_m = 2 * altura_camara_m * np.tan(fov_horizontal_rad / 2)

# Iniciar la cámara
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
ret, frame = cap.read()
if not ret:
    print("No se pudo acceder a la cámara.")
    exit()

frame_height, frame_width = frame.shape[:2]
pixeles_por_metro = frame_width / fov_ancho_m

# Calcular dimensiones del área de 1.20m en píxeles
ancho_px = int(area_real_m * pixeles_por_metro)
alto_px = int(area_real_m * (frame_height / frame_width))  # mantener proporción de aspecto

# Coordenadas para centrar el rectángulo en la imagen
x1 = int((frame_width - ancho_px) / 2)
y1 = int((frame_height - alto_px) / 2)
x2 = x1 + ancho_px
y2 = y1 + alto_px

print(f"📏 Campo de visión estimado: {fov_ancho_m:.2f} m de ancho")
print(f"📐 Area dibujada: {area_real_m} x {area_real_m} m -> {ancho_px} x {alto_px} px")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Dibuja un rectángulo simulando el área de 1.20x1.20 m
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, "Area 1.22m x 1.2m", (x1 + 10, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Verificación de FOV", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
