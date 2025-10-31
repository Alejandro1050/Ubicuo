import time
from gpiozero import DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import busio
import board
from adafruit_vl53l0x import VL53L0X

obstacle_distance = 400

# --- Configuración del hardware ---
Device.pin_factory = LGPIOFactory()

# --- Configuración de los sensores VL53L0X ---
i2c = busio.I2C(board.SCL, board.SDA)

xshut_pins = {
    'frontal': 4,
    'derecha': 27,
    'izquierda': 22
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
    
motor_left_a = DigitalOutputDevice(5)
motor_left_b = DigitalOutputDevice(6)
motor_right_a = DigitalOutputDevice(13)
motor_right_b = DigitalOutputDevice(19)

def brake():
    motor_left_a.on()
    motor_left_b.on()
    motor_right_a.on()
    motor_right_b.on()
    print("BRAKE")
    
def stop():
    motor_left_a.off()
    motor_left_b.off()
    motor_right_a.off()
    motor_right_b.off()
    print("STOP")
    
def right():
    motor_left_a.on()
    motor_left_b.off()
    motor_right_a.off()
    motor_right_b.on()
    print("RIGHT")
    
def left():
    motor_left_a.off()
    motor_left_b.on()
    motor_right_a.on()
    motor_right_b.off()
    print("LEFT")
    
def forward():
    motor_left_a.on()
    motor_left_b.off()
    motor_right_a.on()
    motor_right_b.off()
    print("FORWARD")
    
def back():
    motor_left_a.off()
    motor_left_b.on()
    motor_right_a.off()
    motor_right_b.on()
    print("BACK") 
    
def read_sensors():
    distances = {}
    for name, sensor in sensors.items():
        try:
            distances[name] = sensor.range
        except Exception:
            distances[name] = 1000  # Si falla la lectura, asumir distancia segura
    return distances

def navigate():
    distances = read_sensors()
    dist_front = distances.get('frontal', 1000)
    dist_left = distances.get('izquierda', 1000)
    dist_right = distances.get('derecha', 1000)

    print(f"Distancias: Frontal={dist_front:4d}mm | Izq={dist_left:4d}mm | Der={dist_right:4d}mm")
    
    # Nos movemos al frente
    if dist_front > obstacle_distance: 
        forward()
    
    # Si el sensor de frente y de la izquierda, nos movemos a la derecha
    if dist_front <= obstacle_distance and dist_left <= obstacle_distance:
        brake()
        right()
    
    # Si el sensor de frente y de la derecha, nos movemos a la izquierda
    if dist_front <= obstacle_distance and dist_right <= obstacle_distance:
        brake()
        left()
        
    # Si el sensor de frente, de la derecha y de la izquierda, retrocedemos y doblamos    
    if dist_front <= obstacle_distance and dist_right <= obstacle_distance and dist_left <= obstacle_distance:
        back()
        time.sleep(1)
        left()
    
try:
    forward()
    while True:
        navigate()
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nDeteniendo el sistema...")

finally:
    motor()
    for pin_device in shutdown_pins.values():
        pin_device.off()
    print("Sistema detenido correctamente.")
