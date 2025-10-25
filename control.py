from time import sleep
import pygame
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, AngularServo, PWMOutputDevice, DigitalOutputDevice

# Configuración del hardware
Device.pin_factory = LGPIOFactory()

# === SERVO ===
servo_pin = 19
# AngularServo usa grados (min_angle, max_angle)
servo = AngularServo(19, min_angle=-90, max_angle=90,
                     min_pulse_width=0.0005,
                     max_pulse_width=0.0025)

SERVO_MIN = -45
SERVO_MAX = 45

# === MOTOR ===
motor_adelante = DigitalOutputDevice(6)  # Dirección adelante
motor_atras = DigitalOutputDevice(5)     # Dirección atrás
velocidad_motor = PWMOutputDevice(13)    # PWM velocidad

# Variable global de velocidad
speed = 0

# === FUNCIONES ===

def map_joystick_to_angle(joystick_value):
    """Convierte valor del joystick (-1 a 1) a ángulo del servo (-45 a 45)"""
    angle = joystick_value * ((SERVO_MAX - SERVO_MIN) / 2) + (SERVO_MAX + SERVO_MIN) / 2
    return max(SERVO_MIN, min(SERVO_MAX, angle))

def value_to_speed(joystick_value):
    """Convierte valor del gatillo (-1 a 1) a velocidad (0 a 1)"""
    speed_value = (joystick_value + 1) / 2
    return max(0, min(1, speed_value))

# === Inicialización de pygame y joystick ===
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"Joystick detectado: {joystick.get_name()}")
    print(f"Ejes: {joystick.get_numaxes()}")
    
    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.JOYAXISMOTION:
                    # Eje 0: control del servo (dirección)
                    if event.axis == 0:
                        angle = map_joystick_to_angle(event.value)
                        servo.angle = angle
                        print(f"Servo: {angle:.1f}°")
                    
                    # Eje 5: gatillo RT (velocidad del motor)
                    elif event.axis == 5:
                        speed = value_to_speed(event.value)
                        velocidad_motor.value = speed
                        print(f"Velocidad PWM: {speed:.2f}")
                    
                    # Eje 1: control del movimiento adelante/atrás
                    elif event.axis == 1:
                        if event.value < -0.1:  # Palanca arriba: ADELANTE
                            motor_adelante.on()
                            motor_atras.off()
                            velocidad_motor.value = speed
                            print(f"ADELANTE - Velocidad: {speed:.2f}")
                        elif event.value > 0.1:  # Palanca abajo: ATRÁS
                            motor_adelante.off()
                            motor_atras.on()
                            velocidad_motor.value = speed
                            print(f"ATRÁS - Velocidad: {speed:.2f}")
                        else:  # Palanca centrada: DETENER
                            motor_adelante.off()
                            motor_atras.off()
                            velocidad_motor.value = 0
                            print("Motor DETENIDO")
            
            sleep(0.01)
            
    except KeyboardInterrupt:
        print("Programa interrumpido por el usuario")
    finally:
        # Limpieza
        motor_adelante.off()
        motor_atras.off()
        velocidad_motor.off()
        pygame.quit()
        print("GPIO limpiado y pygame cerrado")

else:
    print("¡No se detectó ningún joystick!")
