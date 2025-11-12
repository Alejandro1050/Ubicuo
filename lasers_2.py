import time
from gpiozero import DigitalOutputDevice
import smbus2
from adafruit_vl53l0x import VL53L0X
import busio
import board

# --- Inicialización del bus I2C de la Raspberry Pi ---
# Para GPIOZero usamos smbus2 para el I2C, pero Adafruit VL53L0X necesita busio
i2c = busio.I2C(board.SCL, board.SDA)  # Mantenemos busio para el sensor VL53L0X

# --- Configuración de los pines XSHUT usando GPIOZero ---
# Usamos números BCM (GPIO) para los pines
xshut_pins = {
    'frontal': 4,    # GPIO 4
    'derecha': 27,   # GPIO 17  
    'izquierda': 22, # GPIO 27
    'trasero': 17     # GPIO 22
}

# Crear objetos DigitalOutputDevice para cada pin XSHUT
shutdown_pins = {}
for name, pin in xshut_pins.items():
    shutdown_pins[name] = DigitalOutputDevice(pin, initial_value=False)
    print(f"Pin {name} (GPIO {pin}) configurado como salida digital")

# --- Inicialización de los sensores ---
sensors = {}
new_addresses = {
    'izquierda': 0x30,
    'frontal': 0x31,
    'derecha': 0x32,
    'trasero': 0x33
}

print("Inicializando sensores...")

# Encender y reasignar dirección a cada sensor, uno por uno
for name, pin_device in shutdown_pins.items():
    # Encender el sensor actual (activar HIGH)
    pin_device.on()
    time.sleep(0.1)  # Darle tiempo para que despierte

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
        # Si falla, apagar el pin para el siguiente intento
        pin_device.off()
        continue

    time.sleep(0.05)  # Pequeña pausa entre inicializaciones

print("\nTodos los sensores inicializados. Tomando lecturas...")

# --- Bucle de Lectura ---
try:
    while True:
        # Leer la distancia de cada sensor y mostrarla
        dist_izq = sensors.get('izquierda').range if 'izquierda' in sensors else -1
        dist_fro = sensors.get('frontal').range if 'frontal' in sensors else -1
        dist_der = sensors.get('derecha').range if 'derecha' in sensors else -1
        dist_tra = sensors.get('trasero').range if 'trasero' in sensors else -1

        print(f"Izquierda: {dist_izq:4d}mm  |  Frontal: {dist_fro:4d}mm  |  Derecha: {dist_der:4d}mm |  Trasero: {dist_tra:4d}mm")

        time.sleep(0.2)  # Pausa entre lecturas

except KeyboardInterrupt:
    print("\nPrograma detenido por el usuario.")
finally:
    # Limpiar los pines al salir - GPIOZero maneja la limpieza automáticamente
    for pin_device in shutdown_pins.values():
        pin_device.close()
    print("Pines GPIO limpiados.")
