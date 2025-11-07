import time
import cv2
import numpy as np
from gpiozero import DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# --- Configuración del hardware ---
Device.pin_factory = LGPIOFactory()

# --- Configuración de los motores ---
motor_left_a = DigitalOutputDevice(5)
motor_left_b = DigitalOutputDevice(6)
motor_right_a = DigitalOutputDevice(13)
motor_right_b = DigitalOutputDevice(19)

# --- Configuración de la cámara ---
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
camera.set(cv2.CAP_PROP_FPS, 30)

# --- Parámetros de seguimiento ---
LINE_COLOR = 'black'  # 'black' o 'white'
MIN_AREA = 500        # Área mínima del contorno para detectar línea
KP = 0.01             # Constante proporcional para control PID básico

def brake():
    motor_left_a.on()
    motor_left_b.on()
    motor_right_a.on()
    motor_right_b.on()

def stop():
    motor_left_a.off()
    motor_left_b.off()
    motor_right_a.off()
    motor_right_b.off()

def right():
    motor_left_a.on()
    motor_left_b.off()
    motor_right_a.off()
    motor_right_b.on()

def left():
    motor_left_a.off()
    motor_left_b.on()
    motor_right_a.on()
    motor_right_b.off()

def forward():
    motor_left_a.on()
    motor_left_b.off()
    motor_right_a.on()
    motor_right_b.off()

def back():
    motor_left_a.off()
    motor_left_b.on()
    motor_right_a.off()
    motor_right_b.on()

def process_frame(frame):
    """Procesa el frame y detecta la línea"""
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Aplicar desenfoque para reducir ruido
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Elegir umbral según el color de la línea
    if LINE_COLOR == 'black':
        _, threshold = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)
    else:
        _, threshold = cv2.threshold(blurred, 175, 255, cv2.THRESH_BINARY)
    
    # Operaciones morfológicas para limpiar la imagen
    kernel = np.ones((5, 5), np.uint8)
    cleaned = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
    
    return cleaned

def find_line_center(binary_image):
    """Encuentra el centro de la línea usando momentos de imagen"""
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Encontrar el contorno más grande
    largest_contour = max(contours, key=cv2.contourArea)
    
    if cv2.contourArea(largest_contour) < MIN_AREA:
        return None
    
    # Calcular momentos para encontrar el centroide
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return None
        
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])
    
    return (center_x, center_y)

def follow_line():
    """Lógica principal de seguimiento de línea"""
    ret, frame = camera.read()
    if not ret:
        print("Error: No se pudo capturar el frame")
        return
    
    # Procesar imagen
    processed = process_frame(frame)
    
    # Encontrar centro de la línea
    line_center = find_line_center(processed)
    
    # Obtener dimensiones de la imagen
    height, width = processed.shape
    image_center = width // 2
    
    if line_center is not None:
        # Calcular error respecto al centro
        error = line_center[0] - image_center
        
        # Control proporcional simple
        if abs(error) < 30:  # Zona muerta pequeña
            forward()
            print(f"AVANZANDO | Error: {error}")
        elif error > 0:
            right()
            print(f"GIRANDO DERECHA | Error: {error}")
        else:
            left()
            print(f"GIRANDO IZQUIERDA | Error: {error}")
            
        # Dibujar información de debug
        cv2.circle(frame, line_center, 5, (0, 255, 0), -1)
        cv2.line(frame, (image_center, 0), (image_center, height), (255, 0, 0), 2)
    else:
        # No se detectó línea - comportamiento de búsqueda
        print("LÍNEA PERDIDA - Buscando...")
        stop()
        time.sleep(0.5)
        # Girar en busca de la línea
        left()
        time.sleep(0.3)
    
    # Mostrar imágenes (opcional, para debug)
    cv2.imshow('Original', frame)
    cv2.imshow('Procesado', processed)
    
    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        raise KeyboardInterrupt

try:
    print("Iniciando seguidor de línea...")
    print("Presiona 'q' en la ventana de imagen para salir")
    
    # Calentamiento de la cámara
    time.sleep(2)
    
    while True:
        follow_line()
        time.sleep(0.05)  # Loop más rápido para mejor respuesta

except KeyboardInterrupt:
    print("\nDeteniendo el sistema...")

finally:
    stop()
    camera.release()
    cv2.destroyAllWindows()
    print("Sistema detenido correctamente.")