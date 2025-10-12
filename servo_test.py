from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory()

from gpiozero import Servo
from time import sleep

# Configuración del servo
SERVO_PIN = 19

# Valores de calibración - AJUSTA ESTOS VALORES SEGÚN TU SERVO
myCorrection = 0.0
maxPW = (2.0 + myCorrection) / 1000  # 2.45 ms
minPW = (1.0 - myCorrection) / 1000  # 0.55 ms

my_servo = Servo(SERVO_PIN, min_pulse_width=minPW, max_pulse_width=maxPW)

def angle_to_value(angle):
    """Convierte ángulo (0-180) a valor del servo (-1 a 1)"""
    return (angle/90.0) - 1

def value_to_angle(value):
    """Convierte valor del servo (-1 a 1) a ángulo (0-180)"""
    return (value+1) * 90 

try:
    print("=== CONTROL DE SERVO POR ÁNGULO ===")
    print("Comandos disponibles:")
    print("  [0-180] - Mover a ángulo específico")
    print("  min - Mover a 0° (mínimo)")
    print("  mid - Mover a 90° (centro)")
    print("  max - Mover a 180° (máximo)")
    print("  calib - Mostrar valores actuales de calibración")
    print("  quit - Salir del programa")
    print()

    while True:
        user_input = input("Ingresa ángulo (0-180) o comando: ").strip().lower()
        
        if user_input == 'quit':
            break
        elif user_input == 'min':
            my_servo.min()
            current_angle = value_to_angle(my_servo.value)
            print(f"Servo movido a: {current_angle:.1f}°")
        elif user_input == 'mid':
            my_servo.mid()
            current_angle = value_to_angle(my_servo.value)
            print(f"Servo movido a: {current_angle:.1f}°")
        elif user_input == 'max':
            my_servo.max()
            current_angle = value_to_angle(my_servo.value)
            print(f"Servo movido a: {current_angle:.1f}°")
        elif user_input == 'calib':
            print(f"Calibración actual: min_pulse_width={minPW*1000:.3f}ms, max_pulse_width={maxPW*1000:.3f}ms")
        else:
            try:
                # Procesar entrada como ángulo
                angle = float(user_input)
                
                if 0 <= angle <= 180:
                    value = angle_to_value(angle)
                    my_servo.value = value
                    current_angle = value_to_angle(my_servo.value)
                    print(f"Servo movido a: {current_angle:.1f}°")
                else:
                    print("Error: El ángulo debe estar entre 0 y 180 grados")
                    
            except ValueError:
                print("Error: Ingresa un número válido (0-180) o un comando reconocido")

except KeyboardInterrupt:
    print("\nPrograma terminado por el usuario")
finally:
    print("GPIO limpiado correctamente")
