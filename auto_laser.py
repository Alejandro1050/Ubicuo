import time
from gpiozero import DigitalOutputDevice, AngularServo, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import busio
import board
from adafruit_vl53l0x import VL53L0X

# --- Configuración del hardware ---
Device.pin_factory = LGPIOFactory()

# --- Configuración de los sensores VL53L0X ---
i2c = busio.I2C(board.SCL, board.SDA)

xshut_pins = {
    'frontal': 4,
    'derecha': 17,
    'izquierda': 27
}

shutdown_pins = {}
for name, pin in xshut_pins.items():
    shutdown_pins[name] = DigitalOutputDevice(pin, initial_value=False)

new_addresses = {
    'izquierda': 0x30,
    'frontal': 0x31,
    'derecha': 0x32
}

sensors = {}
print("Inicializando sensores...")

for name, pin_device in shutdown_pins.items():
    pin_device.on()
    time.sleep(0.1)
    
    try:
        sensor_temp = VL53L0X(i2c)
        new_addr = new_addresses[name]
        sensor_temp.set_address(new_addr)
        sensors[name] = sensor_temp
        print(f"Sensor '{name}' inicializado en {hex(new_addr)}")
    except Exception as e:
        print(f"Error con sensor '{name}': {e}")
        pin_device.off()
    
    time.sleep(0.05)

# --- Configuración del motor y servo ---
servo_pin = 19
servo = AngularServo(
    servo_pin,
    min_angle=-45,
    max_angle=45,
    min_pulse_width=0.0006,
    max_pulse_width=0.0024
)
servo.angle = 0

motor_adelante = DigitalOutputDevice(6)
motor_atras = DigitalOutputDevice(5)
velocidad_motor = PWMOutputDevice(13)

# --- Parámetros ---
OBSTACLE_DISTANCE = 200  # mm
SPEED_NORMAL = 0.5
SPEED_TURN = 0.5

# --- Funciones de movimiento ---
def stop_motor():
    motor_adelante.off()
    motor_atras.off()
    velocidad_motor.value = 0

def move_forward(speed=SPEED_NORMAL):
    motor_adelante.on()
    motor_atras.off()
    velocidad_motor.value = speed
    servo.angle = 0
    print("Avanzando...")

def move_back(duration=1):
    motor_adelante.off()
    motor_atras.on()
    velocidad_motor.value = SPEED_NORMAL
    servo.angle = 0
    print("Retrocediendo...")
    time.sleep(duration)
    stop_motor()

def turn_left():
    servo.angle = -35
    motor_adelante.on()
    motor_atras.off()
    velocidad_motor.value = SPEED_TURN
    print("Girando a la IZQUIERDA")
    time.sleep(1.0)
    stop_motor()

def turn_right():
    servo.angle = 35
    motor_adelante.on()
    motor_atras.off()
    velocidad_motor.value = SPEED_TURN
    print("Girando a la DERECHA")
    time.sleep(1.0)
    stop_motor()

# --- Lectura de sensores ---
def read_sensors():
    distances = {}
    for name, sensor in sensors.items():
        try:
            distances[name] = sensor.range
        except Exception:
            distances[name] = 1000  # Si falla la lectura, asumir distancia segura
    return distances

# --- Sistema de decisión con 3 sensores ---
def navigate():
    distances = read_sensors()
    dist_front = distances.get('frontal', 1000)
    dist_left = distances.get('izquierda', 1000)
    dist_right = distances.get('derecha', 1000)

    print(f"Distancias: Frontal={dist_front:4d}mm | Izq={dist_left:4d}mm | Der={dist_right:4d}mm")

    # Evaluar situación
    if dist_front > OBSTACLE_DISTANCE:
        # Camino despejado
        move_forward(SPEED_NORMAL)

    else:
        # Obstáculo detectado al frente
        stop_motor()
        print("🚧 Obstáculo detectado al frente")

        # Decidir hacia dónde girar
        if dist_left > OBSTACLE_DISTANCE and dist_right > OBSTACLE_DISTANCE:
            # Ambos lados libres → elegir el más amplio
            if dist_left > dist_right:
                turn_left()
            else:
                turn_right()
        elif dist_left > OBSTACLE_DISTANCE:
            turn_left()
        elif dist_right > OBSTACLE_DISTANCE:
            turn_right()
        else:
            # Ambos lados bloqueados → retroceder
            print("🔙 Ambos lados bloqueados, retrocediendo...")
            move_back(duration=1)
            # Luego girar hacia el lado más libre
            if dist_left > dist_right:
                turn_left()
            else:
                turn_right()

# --- Bucle principal ---
print("\nIniciando navegación automática...")
print(f"Umbral de obstáculo: {OBSTACLE_DISTANCE} mm")

try:
    move_forward()
    while True:
        navigate()
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nDeteniendo el sistema...")

finally:
    stop_motor()
    servo.angle = 0
    for pin_device in shutdown_pins.values():
        pin_device.off()
    print("Sistema detenido correctamente.")
