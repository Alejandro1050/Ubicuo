#!/usr/bin/env python3
import RPi.GPIO as GPIO
from time import sleep
import sys

# Configuración del pin del servo
SERVO_PIN = 19

# Configurar el modo BCM (usando números GPIO)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Crear objeto PWM con frecuencia de 50Hz
pwm = GPIO.PWM(SERVO_PIN, 50)

# Función para convertir ángulo a ciclo de trabajo
def set_servo_angle(angle):
    # Asegurar que el ángulo está en el rango válido (0-180)
    angle = max(0, min(180, angle))
    
    # Fórmula para convertir ángulo a duty cycle (rango típico 2%-12%)
    duty_cycle = angle / 18 + 2
    pwm.ChangeDutyCycle(duty_cycle)
    sleep(0.5)  # Dar tiempo al servo para llegar a la posición
    pwm.ChangeDutyCycle(0)  # Detener la señal para evitar vibraciones

try:
    # Iniciar PWM con duty cycle 0
    pwm.start(0)
    print("Control de Servo activado. Escribe 'quit' para salir.")
    
    while True:
        try:
            # Leer entrada del usuario
            user_input = input("Ingresa el ángulo (0-180): ")
            
            if user_input.lower() == 'quit':
                break
                
            angle = int(user_input)
            set_servo_angle(angle)
            print(f"Servo movido a {angle} grados")
            
        except ValueError:
            print("Error: Por favor ingresa un número válido entre 0 y 180")
        except KeyboardInterrupt:
            print("\nPrograma interrumpido por el usuario")
            break

finally:
    # Limpieza garantizada
    pwm.stop()
    GPIO.cleanup()
    print("GPIO limpiado y programa terminado")