from time import sleep
import pygame
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, AngularServo, PWMOutputDevice, DigitalOutputDevice

# === Configuración de hardware ===
Device.pin_factory = LGPIOFactory()

# === SERVO ===
servo_pin = 19
servo = AngularServo(
    servo_pin,
    min_angle=-45,
    max_angle=45,
    min_pulse_width=0.0006,
    max_pulse_width=0.0024
)
servo.angle = 0  # Centrar al inicio

# === MOTOR ===
motor_adelante = DigitalOutputDevice(6)
motor_atras = DigitalOutputDevice(5)
velocidad_motor = PWMOutputDevice(13)

# === Variables globales ===
speed = 0.5  # Velocidad inicial
angle = 0    # Servo centrado

# === Inicialización de pygame ===
pygame.init()
screen = pygame.display.set_mode((400, 200))
pygame.display.set_caption("Control de Raspberry Pi con teclado (WASD)")

running = True
print("Controles:\nW = Adelante\nS = Atrás\nA = Izquierda\nD = Derecha\nESPACIO = Detener\nQ/E = Bajar/Subir velocidad\nESC = Salir")

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Detección de teclas presionadas
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                # Movimiento hacia adelante
                elif event.key == pygame.K_w:
                    motor_adelante.on()
                    motor_atras.off()
                    velocidad_motor.value = speed
                    print(f"ADELANTE - Velocidad: {speed:.2f}")

                # Movimiento hacia atrás
                elif event.key == pygame.K_s:
                    motor_adelante.off()
                    motor_atras.on()
                    velocidad_motor.value = speed
                    print(f"ATRÁS - Velocidad: {speed:.2f}")

                # Giro izquierda
                elif event.key == pygame.K_a:
                    angle = max(-45, angle - 15)
                    servo.angle = angle
                    print(f"Giro IZQUIERDA - Ángulo: {angle:.1f}°")

                # Giro derecha
                elif event.key == pygame.K_d:
                    angle = min(45, angle + 15)
                    servo.angle = angle
                    print(f"Giro DERECHA - Ángulo: {angle:.1f}°")

                # Detener motor
                elif event.key == pygame.K_SPACE:
                    motor_adelante.off()
                    motor_atras.off()
                    velocidad_motor.value = 0
                    print("Motor DETENIDO")

                # Aumentar velocidad
                elif event.key == pygame.K_e:
                    speed = min(1.0, speed + 0.1)
                    print(f"Velocidad aumentada: {speed:.2f}")

                # Disminuir velocidad
                elif event.key == pygame.K_q:
                    speed = max(0.1, speed - 0.1)
                    print(f"Velocidad reducida: {speed:.2f}")

            elif event.type == pygame.KEYUP:
                # Al soltar W o S, detener motor
                if event.key in [pygame.K_w, pygame.K_s]:
                    motor_adelante.off()
                    motor_atras.off()
                    velocidad_motor.value = 0
                    print("Motor DETENIDO")

        sleep(0.05)

except KeyboardInterrupt:
    print("Programa interrumpido por el usuario")

finally:
    # Limpieza
    motor_adelante.off()
    motor_atras.off()
    velocidad_motor.off()
    pygame.quit()
    print("GPIO limpiado y pygame cerrado")
