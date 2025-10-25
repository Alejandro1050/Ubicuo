import time
from gpiozero import DigitalOutputDevice, Servo, PWMOutputDevice
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
    'izquierda': 27,
    'trasero': 22
}

shutdown_pins = {}
for name, pin in xshut_pins.items():
    shutdown_pins[name] = DigitalOutputDevice(pin, initial_value=False)

new_addresses = {
    'izquierda': 0x30,
    'frontal': 0x31,
    'derecha': 0x32,
    'trasero': 0x33
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
servo = Servo(servo_pin, min_pulse_width=0.0005, max_pulse_width=0.0025)

motor_adelante = DigitalOutputDevice(6)
motor_atras = DigitalOutputDevice(5)  
velocidad_motor = PWMOutputDevice(13)

# --- Parámetros ---
OBSTACLE_DISTANCE = 200  # mm
SPEED_NORMAL = 0.5
SPEED_TURN = 0.5

# --- Funciones de movimiento ---
def stop_motor():
    motor_adelante.on()
    motor_atras.on()
    velocidad_motor.value = 0

def move_forward(speed=SPEED_NORMAL):
    stop_motor()
    servo.value = 0
    motor_adelante.on()
    motor_atras.off()
    velocidad_motor.value = speed

def move_back(speed=SPEED_NORMAL):
    stop_motor()
    servo.value = 0
    motor_adelante.off()
    motor_atras.on()
    velocidad_motor.value = speed

def turn_left():
    stop_motor()
    time.sleep(0.3)
    servo.value = -0.5
    motor_adelante.on()
    motor_atras.off()
    velocidad_motor.value = SPEED_TURN
    print("Girando a la IZQUIERDA")

def turn_right():
    stop_motor()
    time.sleep(0.3)
    servo.value = 0.5
    motor_adelante.on()
    motor_atras.off()
    velocidad_motor.value = SPEED_TURN
    print("Girando a la DERECHA")

# --- Nuevo sistema de decisión ---
def avoid_obstacle():
    """Evalúa los sensores y decide la dirección de giro sin memoria."""
    dist_izq = sensors['izquierda'].range if 'izquierda' in sensors else 1000
    dist_der = sensors['derecha'].range if 'derecha' in sensors else 1000
    dist_back = sensors['trasero'].range if 'trasero' in sensors else 1000

    print(f"Obstáculo: Izq={dist_izq}mm, Der={dist_der}mm, Atrás={dist_back}mm")

    # Prioriza el lado más libre
    if dist_izq > OBSTACLE_DISTANCE and dist_der > OBSTACLE_DISTANCE:
        # Ambos lados libres, elige el más amplio
        if dist_izq > dist_der:
            turn_left()
        else:
            turn_right()
    elif dist_izq > OBSTACLE_DISTANCE:
        turn_left()
    elif dist_der > OBSTACLE_DISTANCE:
        turn_right()
    else:
        # Ambos lados bloqueados → retroceder un poco
        print("Ambos lados bloqueados, retrocediendo...")
        move_back()
        time.sleep(1)
        stop_motor()
        # Elegir el lado más libre para salir
        if dist_izq > dist_der:
            turn_left()
        else:
            turn_right()
    
    # Mantener el giro un poco
    time.sleep(1.2)
    stop_motor()

# --- Lectura de sensores ---
def check_sensors():
    distances = {}
    for name, sensor in sensors.items():
        try:
            distances[name] = sensor.range
        except:
            distances[name] = 1000
    return distances

# --- Bucle principal ---
print("\nIniciando navegación automática...")
print(f"Umbral de obstáculo: {OBSTACLE_DISTANCE}mm")

try:
    move_forward()
    while True:
        distances = check_sensors()
        dist_front = distances['frontal']

        print(f"Frontal: {dist_front:4d}mm")

        if dist_front <= OBSTACLE_DISTANCE:
            print("¡Obstáculo detectado!")
            stop_motor()
            avoid_obstacle()
        else:
            move_forward(SPEED_NORMAL)
            servo.value = 0
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nDeteniendo el sistema...")

finally:
    stop_motor()
    servo.value = 0
    for pin_device in shutdown_pins.values():
        pin_device.off()
    print("Sistema detenido correctamente")
