import time
import cv2
import numpy as np
from gpiozero import DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import busio
import board
from adafruit_vl53l0x import VL53L0X
from picamera2 import Picamera2

# --- Configuration ---
TARGET_ARUCO_ID = 25
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_250
MARKER_SIZE_CM = 5  # Actual physical size of your ArUco marker in centimeters
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CENTER_THRESHOLD = 50  # Pixel tolerance for considering marker centered
DETECTION_TIMEOUT = 2.0  # Seconds before considering marker lost

# --- Hardware Configuration ---
Device.pin_factory = LGPIOFactory()
obstacle_distance = 200

# --- VL53L0X Sensor Configuration ---
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
print("Initializing VL53L0X sensors...")

for name, pin_device in shutdown_pins.items():
    pin_device.on()
    time.sleep(0.1)
    
    try:
        sensor_temp = VL53L0X(i2c)
        new_addr = new_addresses[name]
        sensor_temp.set_address(new_addr)
        sensors[name] = sensor_temp
        print(f"Sensor '{name}' initialized at {hex(new_addr)}")
    except Exception as e:
        print(f"Error with sensor '{name}': {e}")
        pin_device.off()
    
    time.sleep(0.05)

# --- Motor Control Configuration ---
motor_left_a = DigitalOutputDevice(5)
motor_left_b = DigitalOutputDevice(6)
motor_right_a = DigitalOutputDevice(13)
motor_right_b = DigitalOutputDevice(19)

# --- Camera Initialization ---
print("Initializing camera...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
time.sleep(2)  # Allow camera to warm up

# --- ArUco Detection Setup ---
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# --- Motor Control Functions ---
def brake():
    motor_left_a.on()
    motor_left_b.on()
    motor_right_a.on()
    motor_right_b.on()

def stop():
    motor_left_a.off()
    motor_left_b.off()
    motor_right_a.off()
    motor_right_b.off()

def right():
    motor_left_a.on()
    motor_left_b.off()
    motor_right_a.off()
    motor_right_b.on()

def left():
    motor_left_a.off()
    motor_left_b.on()
    motor_right_a.on()
    motor_right_b.off()

def forward():
    motor_left_a.on()
    motor_left_b.off()
    motor_right_a.on()
    motor_right_b.off()

def back():
    motor_left_a.off()
    motor_left_b.on()
    motor_right_a.off()
    motor_right_b.on()

def read_sensors():
    distances = {}
    for name, sensor in sensors.items():
        try:
            distances[name] = sensor.range
        except Exception:
            distances[name] = 1000
    return distances

def navigate_obstacles():
    distances = read_sensors()
    dist_front = distances.get('frontal', 1000)
    dist_left = distances.get('izquierda', 1000)
    dist_right = distances.get('derecha', 1000)

    print(f"Distances: Front={dist_front:4d}mm | Left={dist_left:4d}mm | Right={dist_right:4d}mm")
    
    # Priority: obstacle avoidance over ArUco tracking
    if dist_front <= obstacle_distance or dist_left <= obstacle_distance or dist_right <= obstacle_distance:
        if dist_front <= obstacle_distance:
            if dist_left > obstacle_distance:
                left()
            elif dist_right > obstacle_distance:
                right()
            else:
                back()
                time.sleep(1)
        return True  # Obstacle avoidance active
    return False  # No obstacles detected

def follow_aruco_marker():
    # Capture frame from camera
    frame = picam2.capture_array()
    
    # Convert to BGR for OpenCV (camera captures in RGB)
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # Detect ArUco markers
    corners, ids, rejected = detector.detectMarkers(frame_bgr)
    
    target_detected = False
    marker_center_x = None
    
    if ids is not None:
        # Look for our target marker (ID 25)
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == TARGET_ARUCO_ID:
                target_detected = True
                
                # Get marker corners
                marker_corners = corners[i][0]
                
                # Calculate center of marker
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                marker_center_x = center_x
                
                # Calculate marker size in pixels (for distance estimation)
                width_pixels = np.linalg.norm(marker_corners[0] - marker_corners[1])
                
                # Draw detection on frame (for debugging)
                cv2.polylines(frame_bgr, [marker_corners.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(frame_bgr, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame_bgr, f"ID: {marker_id}", 
                           (int(marker_corners[0][0]), int(marker_corners[0][1]) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                print(f"Marker {TARGET_ARUCO_ID} detected at position: {center_x}")
                break
    
    # Display frame (optional - comment out if running headless)
    cv2.imshow("ArUco Detection", frame_bgr)
    cv2.waitKey(1)
    
    return target_detected, marker_center_x

# --- Main Tracking Loop ---
last_detection_time = time.time()
tracking_active = False

print("Starting ArUco tracking system...")
print(f"Looking for marker ID: {TARGET_ARUCO_ID}")

try:
    while True:
        # First priority: obstacle avoidance
        obstacle_detected = navigate_obstacles()
        
        if not obstacle_detected:
            # Second priority: ArUco tracking
            target_detected, marker_x = follow_aruco_marker()
            
            if target_detected:
                last_detection_time = time.time()
                tracking_active = True
                
                # Calculate horizontal position relative to center
                center_screen = CAMERA_WIDTH // 2
                position_diff = marker_x - center_screen
                
                # Decision making based on marker position
                if abs(position_diff) < CENTER_THRESHOLD:
                    # Marker is centered - move forward
                    forward()
                    print("Target centered - Moving FORWARD")
                elif position_diff > 0:
                    # Marker is to the right - turn right
                    right()
                    print("Target to the right - Turning RIGHT")
                else:
                    # Marker is to the left - turn left
                    left()
                    print("Target to the left - Turning LEFT")
                    
            else:
                # Target not detected
                if tracking_active and (time.time() - last_detection_time) > DETECTION_TIMEOUT:
                    # Lost target - stop and search
                    stop()
                    print("Target lost - STOPPING")
                    tracking_active = False
                elif not tracking_active:
                    # Search pattern: slow rotation
                    right()
                    time.sleep(0.5)
                    stop()
                    time.sleep(0.1)
        
        time.sleep(0.1)  # Main loop delay

except KeyboardInterrupt:
    print("\nStopping system...")

finally:
    # Cleanup
    stop()
    cv2.destroyAllWindows()
    picam2.stop()
    for pin_device in shutdown_pins.values():
        pin_device.off()
    print("System stopped correctly.")