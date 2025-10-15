import cv2
import numpy as np
from gpiozero import DigitalOutputDevice, PWMOutputDevice, AngularServo
import time
import warnings
warnings.filterwarnings('ignore')

# Configuración del servo
servo = AngularServo(19, min_angle=-40, max_angle=40)
servo_position = 0

# Configuración del motor
motor_adelante = DigitalOutputDevice(6)
motor_atras = DigitalOutputDevice(5)
velocidad_motor = PWMOutputDevice(13)

# Configuración YOLO - CORREGIDA
def load_yolo():
    try:
        net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
        with open("coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]
        
        layer_names = net.getLayerNames()
        
        # CORRECCIÓN: Manejar diferentes versiones de OpenCV
        unconnected_out_layers = net.getUnconnectedOutLayers()
        
        if hasattr(unconnected_out_layers, 'shape'):
            # Para OpenCV >= 4.5.0
            output_layers = [layer_names[i - 1] for i in unconnected_out_layers.flatten()]
        else:
            # Para OpenCV < 4.5.0
            output_layers = [layer_names[i[0] - 1] for i in unconnected_out_layers]
        
        return net, classes, output_layers
    
    except Exception as e:
        print(f"Error cargando YOLO: {e}")
        print("Asegúrate de tener los archivos:")
        print("- yolov3-tiny.weights")
        print("- yolov3-tiny.cfg") 
        print("- coco.names")
        exit(1)

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
    
    center_x = frame_width // 2
    error = target_x - center_x
    error_normalized = error / center_x
    
    new_position = error_normalized * 40
    new_position = max(-40, min(40, new_position))
    
    if abs(new_position - servo_position) > 2:
        servo.angle = new_position
        servo_position = new_position

# Control del motor
def control_motor(distance, frame_height):
    target_height = frame_height * 0.3
    
    if distance < target_height * 0.8:
        motor_adelante.on()
        motor_atras.off()
        velocidad_motor.value = 0.8
    elif distance > target_height * 1.2:
        motor_adelante.off()
        motor_atras.on()
        velocidad_motor.value = 0.3
    else:
        motor_adelante.off()
        motor_atras.off()
        velocidad_motor.value = 0

def main():
    # Inicializar YOLO
    net, classes, output_layers = load_yolo()
    
    # Inicializar cámara
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    
    print("Sistema de seguimiento iniciado correctamente!")
    print("Presiona 'q' para salir")
    
    # Variable para NMS
    font = cv2.FONT_HERSHEY_PLAIN
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: No se pudo leer el frame de la cámara")
                break
                
            height, width, channels = frame.shape
            
            # Detección de objetos
            blob, outputs = detect_objects(frame, net, output_layers)
            boxes, confidences, class_ids, centers = get_box_dimensions(outputs, height, width)
            
            # Aplicar Non-Maximum Suppression - CORREGIDO
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            
            person_detected = False
            target_center = None
            target_height = 0
            
            if len(indexes) > 0:
                # CORRECCIÓN: Manejar diferentes formatos de indexes
                if hasattr(indexes, 'shape') and len(indexes.shape) > 1:
                    indexes = indexes.flatten()
                
                for i in indexes:
                    if class_ids[i] == 0:
                        x, y, w, h = boxes[i]
                        center_x, center_y = centers[i]
                        
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(frame, f"Persona {confidences[i]:.2f}", 
                                   (x, y - 10), font, 1, (0, 255, 0), 2)
                        
                        if h > target_height:
                            target_height = h
                            target_center = center_x
                            person_detected = True
            
            # Control del robot
            if person_detected and target_center is not None:
                move_servo(target_center, width)
                control_motor(target_height, height)
                
                cv2.putText(frame, f"SEGUIMIENTO: X={target_center}", 
                           (10, 30), font, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Servo: {servo_position:.1f}°", 
                           (10, 60), font, 1, (255, 255, 0), 2)
            else:
                motor_adelante.off()
                motor_atras.off()
                velocidad_motor.value = 0
                cv2.putText(frame, "BUSCANDO PERSONA...", 
                           (10, 30), font, 1, (0, 0, 255), 2)
            
            # Mostrar frame
            cv2.imshow("Seguimiento de Personas - Robot", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Interrupción por teclado")
    except Exception as e:
        print(f"Error durante la ejecución: {e}")
    finally:
        # Limpieza
        cap.release()
        cv2.destroyAllWindows()
        motor_adelante.off()
        motor_atras.off()
        velocidad_motor.value = 0
        servo.angle = 0
        print("Sistema detenido correctamente")

if __name__ == "__main__":
    main()