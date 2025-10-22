import time
import board
import busio
from adafruit_vl53l0x import VL53L0X
import digitalio

# --- Inicialización del bus I2C de la Raspberry Pi ---
i2c = busio.I2C(board.SCL, board.SDA)

# --- Configuración de los pines XSHUT ---
# (Asegúrate de que estos números BCM coincidan con tus conexiones físicas)
xshut_pins = {
    'izquierdo': board.D4,   # GPIO 4
    'frontal':   board.D17,  # GPIO 17
    'derecho':   board.D27   # GPIO 27
}

# Crear objetos DigitalInOut para cada pin XSHUT
shutdown_pins = {}
for name, pin in xshut_pins.items():
    shutdown_pins[name] = digitalio.DigitalInOut(pin)
    shutdown_pins[name].direction = digitalio.Direction.OUTPUT
    shutdown_pins[name].value = False # Apagar todos los sensores al inicio

# --- Inicialización de los sensores ---
sensors = {}
new_addresses = {
    'izquierdo': 0x30,
    'frontal':   0x31,
    'derecho':   0x32
}

print("Inicializando sensores...")

# Encender y reasignar dirección a cada sensor, uno por uno
for name, pin_obj in shutdown_pins.items():
    # Encender el sensor actual
    pin_obj.value = True
    time.sleep(0.1) # Darle tiempo para que despierte

    try:
        # Crear una instancia del sensor en la dirección por defecto (0x29)
        sensor_temp = VL53L0X(i2c)

        # Cambiar su dirección
        new_addr = new_addresses[name]
        sensor_temp.set_address(new_addr)
        print(f"Sensor '{name}' reasignado a la dirección {hex(new_addr)}")

        # Guardar el objeto sensor ya configurado
        sensors[name] = sensor_temp

    except Exception as e:
        print(f"Error inicializando el sensor '{name}': {e}")
        # Si falla, asegurarse de que el pin quede encendido para el siguiente
        continue

print("\nTodos los sensores inicializados. Tomando lecturas...")

# --- Bucle de Lectura ---
try:
    while True:
        # Leer la distancia de cada sensor y mostrarla
        dist_izq = sensors.get('izquierdo').range if 'izquierdo' in sensors else -1
        dist_fro = sensors.get('frontal').range if 'frontal' in sensors else -1
        dist_der = sensors.get('derecho').range if 'derecho' in sensors else -1

        print(f"Izquierdo: {dist_izq:4d}mm  |  Frontal: {dist_fro:4d}mm  |  Derecho: {dist_der:4d}mm")

        time.sleep(0.2) # Pausa entre lecturas

except KeyboardInterrupt:
    print("\nPrograma detenido por el usuario.")
finally:
    # Limpiar los pines al salir
    for pin_obj in shutdown_pins.values():
        pin_obj.deinit()
    print("Pines GPIO limpiados.")
