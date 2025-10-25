import cv2
import numpy as np
from gpiozero import DigitalOutputDevice, PWMOutputDevice, AngularServo
import time
from threading import Thread
import warnings
warnings.filterwarnings('ignore')

# Configuración del servo
servo = AngularServo(19, min_angle=-40, max_angle=40)  # -40° a 40° equivalente a -0.4 a 0.4
servo_position = 0

# Configuración del motor
motor_adelante = DigitalOutputDevice(6)
motor_atras = DigitalOutputDevice(5)
velocidad_motor = PWMOutputDevice(13)

# Configuración YOLO
def load_yolo():
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return net, classes, output_layers

def detect_objects(img, net, output_layers):
    blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outputs = net.forward(output_layers)
    return blob, outputs

def get_box_dimensions(outputs, height, width):
    boxes = []
    confidences = []
    class_ids = []
    centers = []
    
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and class_id == 0:  # class_id 0 es 'persona'
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                centers.append((center_x, center_y))
    
    return boxes, confidences, class_ids, centers

# Control del servo
def move_servo(target_x, frame_width):
    global servo_position
    
    # Calcular error (cuán centrada está la persona)
    center_x = frame_width // 2
    error = target_x - center_x
    error_normalized = error / center_x  # Normalizar entre -1 y 1
    
    # Control PID simple
    new_position = error_normalized * 40  # Escalar a rango del servo
    
    # Limitar el rango del servo
    new_position = max(-40, min(40, new_position))
    
    # Mover servo solo si hay cambio significativo
    if abs(new_position - servo_position) > 2:
        servo.angle = new_position
        servo_position = new_position

# Control del motor
def control_motor(distance, frame_height):
    # distance es la altura del bounding box (proxy de distancia)
    target_height = frame_height * 0.3  # Altura objetivo (30% del frame)
    
    if distance < target_height * 0.8:  # Persona muy lejos
        # Avanzar rápido
        motor_adelante.on()
        motor_atras.off()
        velocidad_motor.value = 0.8  # 80% velocidad
    elif distance > target_height * 1.2:  # Persona muy cerca
        # Retroceder lento
        motor_adelante.off()
        motor_atras.on()
        velocidad_motor.value = 0.3  # 30% velocidad
    else:  # Distancia adecuada
        # Mantener posición
        motor_adelante.off()
        motor_atras.off()
        velocidad_motor.value = 0

def main():
    # Inicializar YOLO
    net, classes, output_layers = load_yolo()
    
    # Inicializar cámara
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # Ancho
    cap.set(4, 480)  # Alto
    
    print("Iniciando sistema de seguimiento...")
    print("Presiona 'q' para salir")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            height, width, channels = frame.shape
            
            # Detección de objetos
            blob, outputs = detect_objects(frame, net, output_layers)
            boxes, confidences, class_ids, centers = get_box_dimensions(outputs, height, width)
            
            # Aplicar Non-Maximum Suppression
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            
            person_detected = False
            target_center = None
            target_height = 0
            
            if len(indexes) > 0:
                for i in indexes.flatten():
                    if class_ids[i] == 0:  # Persona
                        x, y, w, h = boxes[i]
                        center_x, center_y = centers[i]
                        
                        # Dibujar bounding box
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(frame, "Persona", (x, y - 10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Seguir a la persona más grande (más cercana)
                        if h > target_height:
                            target_height = h
                            target_center = center_x
                            person_detected = True
            
            # Control del robot
            if person_detected and target_center is not None:
                # Controlar servo (seguimiento horizontal)
                move_servo(target_center, width)
                
                # Controlar motor (seguimiento de distancia)
                control_motor(target_height, height)
                
                # Mostrar información
                cv2.putText(frame, f"Seguiendo: X={target_center}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                # No detectó persona - detener motor
                motor_adelante.off()
                motor_atras.off()
                velocidad_motor.value = 0
                cv2.putText(frame, "Buscando persona...", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Mostrar frame
            cv2.imshow("Seguimiento de Personas - Robot", frame)
            
            # Salir con 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Interrupción por teclado")
    finally:
        # Limpieza
        cap.release()
        cv2.destroyAllWindows()
        motor_adelante.off()
        motor_atras.off()
        velocidad_motor.value = 0
        servo.angle = 0
        print("Sistema detenido")

if __name__ == "__main__":
    main()
