from time import sleep
import pygame
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Servo, PWMOutputDevice, DigitalOutputDevice

# Configuración del hardware
Device.pin_factory = LGPIOFactory()

# Servo
servo_pin = 19
servo = Servo(servo_pin, min_pulse_width=0.0005, max_pulse_width=0.0025)
SERVO_MIN = -0.4
SERVO_MAX = 0.4

# Motor - configuración corregida
motor_adelante = DigitalOutputDevice(5)  # Control dirección adelante
motor_atras = DigitalOutputDevice(6)     # Control dirección atrás  
velocidad_motor = PWMOutputDevice(13)    # Control de velocidad PWM

# Variable global para la velocidad
speed = 0

def map_joystick_to_servo(joystick_value):
    """Convierte valor del joystick (-1 a 1) al rango limitado del servo (-0.4 a 0.4)"""
    servo_value = joystick_value * ((SERVO_MAX - SERVO_MIN) / 2) + (SERVO_MAX + SERVO_MIN) / 2
    return max(SERVO_MIN, min(SERVO_MAX, servo_value))

def value_to_speed(joystick_value):
    """Convierte valor del gatillo (-1 a 1) a velocidad (0 a 1) para el motor"""
    # El gatillo RT normalmente va de -1 (reposo) a 1 (presionado)
    speed_value = (joystick_value + 1) / 2  # Convierte a rango 0-1
    return max(0, min(1, speed_value))

# Inicializar pygame y joystick
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
                    # Eje 0 (palanca izquierda horizontal) para controlar el SERVO
                    if event.axis == 0:
                        limited_value = map_joystick_to_servo(event.value)
                        servo.value = limited_value
                        print(f"Servo: {limited_value:.2f}")
                    
                    # Eje 5 (gatillo RT) para controlar VELOCIDAD del motor
                    elif event.axis == 5:
                        speed = value_to_speed(event.value)
                        velocidad_motor.value = speed  # Aplicar PWM
                        print(f"Velocidad: {speed:.2f}")
                    
                    # Eje 1 (palanca izquierda vertical) para controlar DIRECCIÓN del motor
                    elif event.axis == 1:
                        if event.value < -0.1:  # Palanca arriba - ADELANTE
                            motor_adelante.on()
                            motor_atras.off()
                            velocidad_motor.value = speed  # Mantener velocidad
                            print(f"ADELANTE - Velocidad: {speed:.2f}")
                        elif event.value > 0.1:  # Palanca abajo - ATRÁS
                            motor_adelante.off()
                            motor_atras.on()
                            velocidad_motor.value = speed  # Mantener velocidad
                            print(f"ATRÁS - Velocidad: {speed:.2f}")
                        else:  # Palanca centrada - DETENER
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