#!/usr/bin/env python
  
'''
Welcome to the ArUco Marker Detector!
  
This program:
  - Detects ArUco markers using OpenCV and Python
  - Compatible with both old and new OpenCV versions
'''
  
from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
import sys
 
# Project: ArUco Marker Detector
desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

def detect_aruco_markers_modern(frame, detector):
    """Detect ArUco markers using modern OpenCV API"""
    corners, ids, rejected = detector.detectMarkers(frame)
    return corners, ids, rejected

def detect_aruco_markers_legacy(frame, aruco_dict, parameters):
    """Detect ArUco markers using legacy OpenCV API"""
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    return corners, ids, rejected
  
def main():
  """
  Main method of the program.
  """
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
      desired_aruco_dictionary))
    sys.exit(0)
  
  # Detect OpenCV version and use appropriate API
  opencv_version = cv2.__version__
  print(f"[INFO] OpenCV version: {opencv_version}")
  print("[INFO] detecting '{}' markers...".format(desired_aruco_dictionary))
  
  # Initialize variables for both APIs
  detector = None
  aruco_dict_legacy = None
  parameters_legacy = None
  use_modern_api = False
  
  # Try modern API first, fall back to legacy if needed
  try:
    # Modern OpenCV API (4.7.0+)
    aruco_dict_modern = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
    parameters_modern = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict_modern, parameters_modern)
    use_modern_api = True
    print("[INFO] Using modern OpenCV ArUco API")
  except AttributeError:
    # Legacy OpenCV API
    try:
      aruco_dict_legacy = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
      parameters_legacy = cv2.aruco.DetectorParameters_create()
      use_modern_api = False
      print("[INFO] Using legacy OpenCV ArUco API")
    except Exception as e:
      print(f"[ERROR] Neither modern nor legacy ArUco API is available: {e}")
      return
   
  # Start the video stream
  cap = cv2.VideoCapture(0)
  
  if not cap.isOpened():
    print("[ERROR] Could not open video device")
    return
  
  print("[INFO] Press 'q' to quit, 's' to save current frame")
   
  while True:
    ret, frame = cap.read()
    
    if not ret:
      print("[ERROR] Failed to capture frame")
      break
     
    # Detect ArUco markers using the appropriate API
    if use_modern_api:
      corners, ids, rejected = detect_aruco_markers_modern(frame, detector)
    else:
      corners, ids, rejected = detect_aruco_markers_legacy(frame, aruco_dict_legacy, parameters_legacy)
       
    # Check that at least one ArUco marker was detected
    if ids is not None and len(corners) > 0:
      ids = ids.flatten()
       
      for (marker_corner, marker_id) in zip(corners, ids):
        corners_reshaped = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners_reshaped
         
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
         
        # Draw bounding box
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
         
        # Draw center
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
         
        # Draw ID
        cv2.putText(frame, str(marker_id), 
          (top_left[0], top_left[1] - 15),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5, (0, 255, 0), 2)
  
    # Display frame
    cv2.imshow('ArUco Marker Detector', frame)
          
    # Handle key presses
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
      break
    elif key == ord('s'):
      # Save current frame
      filename = "aruco_capture.jpg"
      cv2.imwrite(filename, frame)
      print(f"[INFO] Frame saved as {filename}")
  
  # Cleanup
  cap.release()
  cv2.destroyAllWindows()
  print("[INFO] Video stream closed")
   
if __name__ == '__main__':
  print(__doc__)
  main()