import time
import cv2
import numpy as np
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# --- Configuración del hardware ---
Device.pin_factory = LGPIOFactory()

# --- Configuración de los motores con PWM ---
# Configuración H-bridge con PWM para control de velocidad
motor_left_pwm = PWMOutputDevice(5)    # PWM para motor izquierdo adelante
motor_left_dir = DigitalOutputDevice(6) # Dirección motor izquierdo
motor_right_pwm = PWMOutputDevice(13)   # PWM para motor derecho adelante  
motor_right_dir = DigitalOutputDevice(19) # Dirección motor derecho

# --- Configuración de la cámara ---
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
camera.set(cv2.CAP_PROP_FPS, 30)

# --- Parámetros de seguimiento ---
LINE_COLOR = 'black'  # 'black' o 'white'
MIN_AREA = 500        # Área mínima del contorno para detectar línea

# --- Parámetros del control PID ---
KP = 0.8              # Constante proporcional (más agresiva)
KI = 0.001            # Constante integral (para corrección de deriva)
KD = 2.5              # Constante derivativa (para suavizar movimientos)
BASE_SPEED = 0.6      # Velocidad base (0-1)
MAX_SPEED = 0.8       # Velocidad máxima (0-1)
MIN_SPEED = 0.3       # Velocidad mínima (0-1)

# Variables del PID
previous_error = 0
integral = 0
last_line_time = time.time()
line_lost = False

def set_motor_speeds(left_speed, right_speed):
    """Controla la velocidad y dirección de los motores usando PWM"""
    # Limitar velocidades al rango permitido
    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
    
    # Motor izquierdo
    if left_speed >= 0:
        motor_left_dir.off()  # Adelante
        motor_left_pwm.value = left_speed
    else:
        motor_left_dir.on()   # Atrás
        motor_left_pwm.value = -left_speed
    
    # Motor derecho
    if right_speed >= 0:
        motor_right_dir.off()  # Adelante
        motor_right_pwm.value = right_speed
    else:
        motor_right_dir.on()   # Atrás  
        motor_right_pwm.value = -right_speed

def smooth_stop():
    """Detención suave de los motores"""
    motor_left_pwm.value = 0
    motor_right_pwm.value = 0

def smooth_forward(speed_factor=1.0):
    """Movimiento hacia adelante con velocidad controlada"""
    speed = BASE_SPEED * speed_factor
    set_motor_speeds(speed, speed)

def smooth_turn(error, max_turn_speed=0.7):
    """Giro suave basado en el error del PID"""
    # Calcular corrección PID
    global previous_error, integral
    
    proportional = error
    integral += error
    derivative = error - previous_error
    
    # Limitar el término integral para evitar windup
    integral = max(-100, min(100, integral))
    
    # Calcular corrección total
    correction = KP * proportional + KI * integral + KD * derivative
    
    # Aplicar velocidad diferencial a los motores
    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction
    
    # Limitar velocidades
    left_speed = max(MIN_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(MIN_SPEED, min(MAX_SPEED, right_speed))
    
    previous_error = error
    return left_speed, right_speed, correction

def search_line():
    """Comportamiento de búsqueda cuando se pierde la línea"""
    global last_line_time, line_lost
    
    current_time = time.time()
    search_duration = current_time - last_line_time
    
    if search_duration < 1.0:
        # Giro suave inicial
        set_motor_speeds(-0.4, 0.4)  # Giro en el lugar
    elif search_duration < 2.0:
        # Cambio de dirección
        set_motor_speeds(0.4, -0.4)
    else:
        # Búsqueda más agresiva
        set_motor_speeds(0.3, -0.5)
    
    line_lost = True
    return "BÚSQUEDA"

def process_frame(frame):
    """Procesa el frame y detecta la línea con mejoras"""
    # ROI (Region of Interest) - enfocarse en la parte inferior
    height, width = frame.shape[:2]
    roi = frame[int(height*0.4):, :]
    
    # Convertir a escala de grises
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Aplicar desenfoque adaptativo
    blur_size = 5
    blurred = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
    
    # Detección de bordes para mejor precisión en líneas difusas
    edges = cv2.Canny(blurred, 50, 150)
    
    # Elegir umbral según el color de la línea
    if LINE_COLOR == 'black':
        _, threshold = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY_INV)
    else:
        _, threshold = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
    
    # Combinar detección de bordes con threshold
    combined = cv2.bitwise_or(threshold, edges)
    
    # Operaciones morfológicas mejoradas
    kernel_open = np.ones((3, 3), np.uint8)
    kernel_close = np.ones((7, 7), np.uint8)
    
    cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel_open)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel_close)
    
    return cleaned, roi

def find_line_center(binary_image):
    """Encuentra el centro de la línea usando múltiples métodos"""
    global last_line_time, line_lost
    
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Filtrar contornos por área y relación de aspecto
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
            
        # Calcular relación de aspecto del bounding rect
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / h if h > 0 else 0
        
        # Filtrar por relación de aspecto (líneas típicamente alargadas)
        if 0.2 < aspect_ratio < 5.0:
            valid_contours.append(contour)
    
    if not valid_contours:
        return None
    
    # Usar el contorno más grande y centrado
    largest_contour = max(valid_contours, key=cv2.contourArea)
    
    # Método 1: Centroide
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return None
        
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])
    
    # Método 2: Punto más bajo del contorno (para curvas)
    bottommost = tuple(largest_contour[largest_contour[:, :, 1].argmax()][0])
    
    # Combinar ambos métodos (peso mayor al centroide)
    combined_x = int(center_x * 0.7 + bottommost[0] * 0.3)
    
    last_line_time = time.time()
    line_lost = False
    
    return (combined_x, center_y)

def follow_line():
    """Lógica principal de seguimiento de línea mejorada"""
    global line_lost
    
    ret, frame = camera.read()
    if not ret:
        print("Error: No se pudo capturar el frame")
        return "ERROR"
    
    # Procesar imagen
    processed, roi = process_frame(frame)
    
    # Encontrar centro de la línea
    line_center = find_line_center(processed)
    
    # Obtener dimensiones de la ROI
    height, width = roi.shape[:2]
    image_center = width // 2
    
    if line_center is not None:
        # Calcular error normalizado (-1 a 1)
        error = (line_center[0] - image_center) / image_center
        
        # Aplicar filtro de suavizado al error
        smoothed_error = error * 0.7 + previous_error * 0.3
        
        # Zona muerta pequeña para micro-vibraciones
        if abs(smoothed_error) < 0.05:  # ±5% del centro
            smooth_forward(1.0)
            status = f"AVANZANDO | Error: {smoothed_error:.3f}"
        else:
            # Giro suave con PID
            left_speed, right_speed, correction = smooth_turn(smoothed_error)
            set_motor_speeds(left_speed, right_speed)
            status = f"GIRO | Error: {smoothed_error:.3f} | Corrección: {correction:.3f}"
            
        # Dibujar información de debug
        cv2.circle(roi, line_center, 5, (0, 255, 0), -1)
        cv2.line(roi, (image_center, 0), (image_center, height), (255, 0, 0), 2)
        cv2.putText(roi, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
    else:
        # Comportamiento cuando se pierde la línea
        status = search_line()
    
    # Mostrar imágenes (opcional, para debug)
    cv2.imshow('Original', frame)
    cv2.imshow('Procesado', processed)
    
    print(status)
    
    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        raise KeyboardInterrupt
        
    return status

# Función de calibración inicial
def calibrate_line_detection():
    """Función para calibrar los parámetros de detección"""
    print("Calibrando detección de línea...")
    ret, frame = camera.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.mean(gray)
        print(f"Brillo promedio: {avg_brightness}")
        
        # Ajustar umbral automáticamente basado en condiciones de luz
        global LINE_COLOR
        if avg_brightness < 100:
            LINE_COLOR = 'white'
            print("Cambiando a detección de línea blanca")
        else:
            LINE_COLOR = 'black'
            print("Manteniendo detección de línea negra")

try:
    print("Iniciando seguidor de línea mejorado...")
    print("Presiona 'q' en la ventana de imagen para salir")
    
    # Calentamiento y calibración
    time.sleep(2)
    calibrate_line_detection()
    
    # Bucle principal
    while True:
        follow_line()
        time.sleep(0.02)  # Loop a 50Hz para respuesta más rápida

except KeyboardInterrupt:
    print("\nDeteniendo el sistema...")

finally:
    smooth_stop()
    camera.release()
    cv2.destroyAllWindows()
    print("Sistema detenido correctamente.")