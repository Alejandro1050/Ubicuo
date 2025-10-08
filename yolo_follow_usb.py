
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO + ByteTrack en Raspberry Pi 5 (USB cam) — dentro de un programa Python
---------------------------------------------------------------------------
- Soporta modelo .pt (PyTorch) o carpeta exportada NCNN (yolo11n_ncnn_model/).
- Hace tracking con ByteTrack (IDs persistentes).
- Calcula señales de control simples (speed/turn) a partir de la caja objetivo.
- Muestra ventana con overlay; imprime dx/area/speed/turn en consola.
- Predeterminado: clase 'person' (id=0).

Uso típico (primero exporta a NCNN para subir FPS en CPU ARM):
  pip install ultralytics opencv-python
  yolo export model=yolo11n.pt format=ncnn
  python yolo_follow_usb.py --model yolo11n_ncnn_model --imgsz 480 --conf 0.4 --source 0

Si no exportaste a NCNN, también funciona con el .pt (algo más lento en Pi 5):
  python yolo_follow_usb.py --model yolov8n.pt --imgsz 480 --conf 0.4 --source 0

Teclas: ESC/q para salir.
"""

import argparse, time, math
import cv2
import numpy as np
from ultralytics import YOLO

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=str, required=True,
                    help="Ruta al modelo (.pt) o carpeta NCNN (p.ej., yolo11n_ncnn_model)")
    ap.add_argument("--source", type=str, default="0",
                    help="0 para USB cam o ruta a video (MP4)")
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--class_id", type=int, default=0, help="Clase a rastrear (0=person)")
    ap.add_argument("--no_display", action="store_true", help="No abrir ventana (solo imprimir)")
    return ap.parse_args()

def clamp(x, lo, hi): return max(lo, min(hi, x))

def main():
    args = parse_args()

    # Cargar modelo (detecta backend por tipo de artifact: .pt vs carpeta NCNN)
    try:
        model = YOLO(args.model)
    except Exception as e:
        print(f"[ERROR] No pude cargar el modelo '{args.model}'. Detalle: {e}")
        return

    # Fuente
    source = 0 if (args.source.isdigit() and len(args.source)<=3) else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print("[ERROR] No se pudo abrir la fuente. Prueba con --source path.mp4")
        return

    # Intentar fijar resolución de la cámara USB
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # Control (parametrización básica)
    kx, kd = 0.8, 1.2     # ganancias de giro y avance
    v_max, w_max = 0.7, 0.6
    dead_x, dead_a = 0.05, 0.01
    a_ref = 0.10          # área objetivo ~ distancia deseada (10% del frame)

    dx_f, a_f, alpha = 0.0, 0.0, 0.5  # suavizado exponencial

    # FPS
    t0 = time.time(); n = 0

    print("[INFO] Presiona ESC o 'q' para salir.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        H, W = frame.shape[:2]

        # Tracking en un frame (persist=True mantiene estado del tracker)
        results = model.track(frame, imgsz=args.imgsz, conf=args.conf,
                              classes=[args.class_id], tracker='bytetrack.yaml',
                              persist=True, verbose=False)
        result = results[0] if results else None

        target = None
        if result is not None and result.boxes is not None and len(result.boxes) > 0:
            # Elegir objetivo: caja con mayor área ponderada por cercanía al centro
            best_score = -1.0
            for b in result.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                w, h = x2 - x1, y2 - y1
                if w <= 0 or h <= 0: continue
                cx, cy = x1 + w//2, y1 + h//2
                area_rel = (w*h) / (W*H + 1e-6)
                center_bonus = 1.0 - abs((cx - W/2) / (W/2))
                score = 0.7*area_rel + 0.3*center_bonus
                if score > best_score:
                    best_score = score
                    target = (cx, cy, w, h)

        # Control (no acciona motores reales; imprime señales)
        speed, turn = 0.0, 0.0
        if target is not None:
            cx, cy, ww, hh = target
            dx = (cx - W/2) / (W/2)            # -1..1 (neg=izq, pos=der)
            a  = (ww*hh) / (W*H + 1e-6)        # 0..1 (proxy de distancia)

            # Suavizado
            dx_f = alpha*dx + (1-alpha)*dx_f
            a_f  = alpha*a  + (1-alpha)*a_f

            # Histéresis / zonas muertas
            if abs(dx_f) < dead_x: dx_f = 0.0
            err_a = (a_ref - a_f)
            if abs(err_a) < dead_a: err_a = 0.0

            # Señales
            turn  = clamp(kx * dx_f, -0.6, 0.6)
            speed = clamp(kd * err_a, -0.7, 0.7)

            # Overlay
            if not args.no_display:
                x1, y1, x2, y2 = cx - ww//2, cy - hh//2, cx + ww//2, cy + hh//2
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.line(frame, (W//2, 0), (W//2, H), (255,255,0), 1)
                cv2.putText(frame, f"dx={dx_f:.2f} a={a_f:.3f} spd={speed:.2f} turn={turn:.2f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        else:
            if not args.no_display:
                cv2.putText(frame, "No target", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # Mostrar (o solo imprimir)
        n += 1
        if n % 15 == 0:
            dt = time.time() - t0
            fps = 15.0 / dt if dt > 0 else 0.0
            t0 = time.time()
            print(f"FPS~{fps:5.1f} | speed={speed:+.2f} turn={turn:+.2f} | dx={dx_f:+.2f} area={a_f:.3f}")

        if not args.no_display:
            cv2.imshow("YOLO + ByteTrack (USB cam)", frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break

    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
