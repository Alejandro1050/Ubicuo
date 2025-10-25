from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory()

from gpiozero import Servo
from time import sleep

# Configuración del servo
SERVO_PIN = 19
# Ajusta estos valores según las especificaciones de tu servo
my_servo = Servo(SERVO_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)

try:
    print("Control de Servo activado (gpiozero). Escribe 'quit' para salir.")

    while True:
        user_input = input("Ingresa el ángulo (-1 para 0°, 0 para 90°, 1 para 180°): ")
        
        if user_input.lower() == 'quit':
            break
            
        try:
            # Convierte la entrada a un valor entre -1 y 1
            value = float(user_input)
            value = max(-1, min(1, value))  # Limita el valor al rango [-1, 1]
            
            my_servo.value = value
            print(f"Servo movido a la posición: {value}")
            
        except ValueError:
            print("Error: Por favor ingresa un número válido.")

except KeyboardInterrupt:
    print("\nPrograma terminado por el usuario.")
finally:
    # El manejo de la limpieza de GPIO es automático con gpiozero
    print("GPIO limpiado correctamente.")
