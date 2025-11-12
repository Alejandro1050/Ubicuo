import cv2
import numpy as np

# --- Configuración ---
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_250
MARKER_SIZE_CM = 5.0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# --- Inicialización de la cámara ---
def initialize_camera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    if not cap.isOpened():
        print("Error: No se puede acceder a la cámara")
        return None
    
    return cap

# --- Configuración de detección ArUco ---
def setup_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    return detector

# --- Calcular información del marcador ---
def calculate_marker_info(corners):
    # Calcular centro
    center_x = int(np.mean(corners[:, 0]))
    center_y = int(np.mean(corners[:, 1]))
    
    # Calcular tamaño en píxeles
    width_px = np.linalg.norm(corners[0] - corners[1])
    height_px = np.linalg.norm(corners[1] - corners[2])
    size_px = (width_px + height_px) / 2
    
    # Calcular área
    area = cv2.contourArea(corners.astype(np.float32))
    
    return center_x, center_y, size_px, area

# --- Dibujar información de todos los marcadores ---
def draw_markers_info(frame, corners_list, ids_list):
    colors = [
        (0, 255, 0),    # Verde
        (255, 0, 0),    # Azul
        (0, 0, 255),    # Rojo
        (255, 255, 0),  # Cian
        (255, 0, 255),  # Magenta
        (0, 255, 255),  # Amarillo
        (128, 0, 128),  # Púrpura
        (0, 128, 128)   # Verde azulado
    ]
    
    for i, (corners, marker_id) in enumerate(zip(corners_list, ids_list.flatten())):
        color = colors[i % len(colors)]
        
        # Dibujar contorno del marcador
        cv2.polylines(frame, [corners.astype(int)], True, color, 3)
        
        # Calcular información
        center_x, center_y, size_px, area = calculate_marker_info(corners)
        
        # Dibujar punto central
        cv2.circle(frame, (center_x, center_y), 8, color, -1)
        
        # Dibujar información de texto
        info_text = f"ID: {marker_id}"
        cv2.putText(frame, info_text, 
                   (int(corners[0][0]) - 40, int(corners[0][1]) - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Información de posición
        pos_text = f"Pos: ({center_x}, {center_y})"
        cv2.putText(frame, pos_text, 
                   (int(corners[0][0]) - 40, int(corners[0][1]) - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Información de tamaño
        size_text = f"Tam: {size_px:.1f}px"
        cv2.putText(frame, size_text, 
                   (int(corners[0][0]) - 40, int(corners[0][1]) + 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Información de área
        area_text = f"Area: {area:.1f}"
        cv2.putText(frame, area_text, 
                   (int(corners[0][0]) - 40, int(corners[0][1]) + 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

# --- Función principal de detección ---
def detect_all_aruco_markers():
    # Inicializar cámara y detector
    cap = initialize_camera()
    if cap is None:
        return
    
    detector = setup_aruco_detector()
    
    print("Iniciando detección de TODOS los marcadores ArUco")
    print("Presiona 'q' para salir")
    print("Presiona 's' para guardar frame actual")
    print("Presiona 'c' para mostrar/ocultar información detallada")
    
    detection_count = 0
    show_detailed_info = True
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se puede leer el frame de la cámara")
            break
        
        # Voltear frame horizontalmente para efecto espejo (opcional)
        frame = cv2.flip(frame, 1)
        
        # Convertir a escala de grises para detección
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detectar todos los marcadores ArUco
        corners, ids, rejected = detector.detectMarkers(gray)
        
        markers_detected = False
        
        if ids is not None:
            markers_detected = True
            detection_count += 1
            
            # Dibujar información de todos los marcadores detectados
            draw_markers_info(frame, corners, ids)
            
            # Mostrar información en consola
            detected_ids = ids.flatten()
            print(f"Marcadores detectados: {len(detected_ids)} - IDs: {list(detected_ids)}")
            
            # Mostrar información resumida en pantalla
            if show_detailed_info:
                info_y = 30
                cv2.putText(frame, f"Marcadores detectados: {len(detected_ids)}", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                info_y += 30
                
                for marker_id in detected_ids:
                    cv2.putText(frame, f"ID: {marker_id}", 
                               (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    info_y += 25
        
        if not markers_detected:
            # Mostrar mensaje cuando no se detectan marcadores
            cv2.putText(frame, "No se detectaron marcadores ArUco", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Mostrar información general en pantalla
        cv2.putText(frame, f"Total detecciones: {detection_count}", 
                   (10, CAMERA_HEIGHT - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        controls_text = "Controles: 'q'=Salir, 's'=Guardar, 'c'=Info"
        cv2.putText(frame, controls_text, 
                   (10, CAMERA_HEIGHT - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Mostrar frame
        cv2.imshow("Deteccion de TODOS los marcadores ArUco", frame)
        
        # Manejar teclas
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Guardar frame actual
            filename = f"aruco_detection_all_{detection_count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Frame guardado como: {filename}")
        elif key == ord('c'):
            show_detailed_info = not show_detailed_info
            print(f"Información detallada: {'ACTIVADA' if show_detailed_info else 'DESACTIVADA'}")
    
    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()
    print(f"\nDetección finalizada. Total de frames con detecciones: {detection_count}")

# --- Versión alternativa: Detectar múltiples diccionarios ---
def detect_multiple_dictionaries():
    """
    Versión que detecta marcadores de diferentes diccionarios ArUco
    """
    # Diccionarios disponibles
    dictionaries = {
        '4X4_50': cv2.aruco.DICT_4X4_50,
        '4X4_100': cv2.aruco.DICT_4X4_100,
        '4X4_250': cv2.aruco.DICT_4X4_250,
        '4X4_1000': cv2.aruco.DICT_4X4_1000,
        '5X5_50': cv2.aruco.DICT_5X5_50,
        '5X5_100': cv2.aruco.DICT_5X5_100,
        '5X5_250': cv2.aruco.DICT_5X5_250,
        '5X5_1000': cv2.aruco.DICT_5X5_1000,
        '6X6_50': cv2.aruco.DICT_6X6_50,
        '6X6_100': cv2.aruco.DICT_6X6_100,
        '6X6_250': cv2.aruco.DICT_6X6_250,
        '6X6_1000': cv2.aruco.DICT_6X6_1000,
        '7X7_50': cv2.aruco.DICT_7X7_50,
        '7X7_100': cv2.aruco.DICT_7X7_100,
        '7X7_250': cv2.aruco.DICT_7X7_250,
        '7X7_1000': cv2.aruco.DICT_7X7_1000
    }
    
    cap = initialize_camera()
    if cap is None:
        return
    
    detectors = {}
    for name, dict_type in dictionaries.items():
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        aruco_params = cv2.aruco.DetectorParameters()
        detectors[name] = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    print("Detección de múltiples diccionarios ArUco activada")
    print("Presiona 'q' para salir")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        all_detections = []
        
        # Probar todos los diccionarios
        for dict_name, detector in detectors.items():
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    all_detections.append({
                        'dict_name': dict_name,
                        'marker_id': marker_id,
                        'corners': corners[i][0]
                    })
        
        # Dibujar todas las detecciones
        for detection in all_detections:
            corners = detection['corners']
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))
            
            # Dibujar contorno
            cv2.polylines(frame, [corners.astype(int)], True, (0, 255, 0), 2)
            
            # Dibujar información
            info_text = f"{detection['dict_name']} ID:{detection['marker_id']}"
            cv2.putText(frame, info_text, 
                       (center_x - 60, center_y - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Mostrar estadísticas
        cv2.putText(frame, f"Marcadores detectados: {len(all_detections)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Deteccion Multi-Diccionario ArUco", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Ejecutar detección simple de todos los marcadores
    #detect_all_aruco_markers()
    
    # Para probar múltiples diccionarios, descomenta la siguiente línea:
    detect_multiple_dictionaries()