"""
vision.py
Pipeline detection (classical CV) and worker safety detection (YOLOv8).
"""

import cv2
import numpy as np

try:
    import torch
    # Pipeline tracking uses the custom YOLOv5s model
    _yolo_pipe = torch.hub.load("ultralytics/yolov5", "custom",
                                path="models/pipeline_tracking_yolov5s.pt",
                                force_reload=False, verbose=False)
    _yolo_pipe.conf = 0.4
    _yolo_pipe.iou  = 0.45
    PIPE_MODEL_AVAILABLE = True
except Exception as e:
    PIPE_MODEL_AVAILABLE = False
    print(f"WARN: YOLOv5 pipe model not loaded ({e}). Using classical CV only.")

try:
    from ultralytics import YOLO
    _yolo_worker = YOLO("models/worker_detection.pt")
    WORKER_MODEL_AVAILABLE = True
except Exception:
    WORKER_MODEL_AVAILABLE = False
    print("WARN: YOLOv8 worker model not found. Worker detection disabled.")


# ── Pipe detection ────────────────────────────────────────────────────────────

def detect_pipe(frame: np.ndarray) -> np.ndarray:
    """
    Detects pipeline boundaries using edge detection + Hough transform.
    Annotates frame with detected boundary lines and estimated centerline.

    Pipeline:
        1. Grayscale + Gaussian blur (sigma=1.5)
        2. Canny edge detection (T_low=50, T_high=150)
        3. Probabilistic Hough transform
        4. Centerline estimation from dominant rho-peaks

    Returns annotated frame.
    """
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
    edges   = cv2.Canny(blurred, 50, 150)

    lines = cv2.HoughLinesP(edges,
                             rho=1,
                             theta=np.pi / 180,
                             threshold=80,
                             minLineLength=100,
                             maxLineGap=20)

    h, w = frame.shape[:2]
    rho_values = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Draw detected boundary lines in cyan
            cv2.line(frame, (x1, y1), (x2, y2), (255, 220, 0), 1)
            # Compute rho for this line
            theta = np.arctan2(y2 - y1, x2 - x1)
            rho   = x1 * np.cos(theta) + y1 * np.sin(theta)
            rho_values.append(rho)

    # Estimate centerline from mean of rho values
    if len(rho_values) >= 2:
        rho_values.sort()
        left_rho  = np.mean(rho_values[:len(rho_values)//2])
        right_rho = np.mean(rho_values[len(rho_values)//2:])
        center_x  = int((left_rho + right_rho) / 2)
        cv2.line(frame, (center_x, 0), (center_x, h), (0, 255, 0), 2)
        cv2.putText(frame, "Pipe center", (center_x + 5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # YOLOv5s pipe detection overlay
    if PIPE_MODEL_AVAILABLE:
        results = _yolo_pipe(frame, size=320)
        for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
            x1, y1, x2, y2 = map(int, xyxy)
            label = f"pipe {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 1)
            cv2.putText(frame, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

    return frame


# ── Worker detection ──────────────────────────────────────────────────────────

def detect_workers(frame: np.ndarray) -> tuple[np.ndarray, bool]:
    """
    Detects human presence and classifies posture using YOLOv8n.
    INT8 quantized model for real-time inference on Raspberry Pi (~220ms).

    Posture classification:
        - Bounding box aspect ratio (h/w) > 1.8  => upright
        - Bounding box aspect ratio (h/w) <= 1.8 => prone / lying down

    Returns:
        annotated frame, worker_alert (bool: True if prone worker detected)
    """
    if not WORKER_MODEL_AVAILABLE:
        return frame, False

    alert = False
    results = _yolo_worker(frame, imgsz=320, conf=0.5, verbose=False)

    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf  = float(box.conf[0])
            bw    = x2 - x1
            bh    = y2 - y1
            ratio = bh / max(bw, 1)

            posture = "Upright" if ratio > 1.8 else "PRONE"
            color   = (0, 255, 0) if posture == "Upright" else (0, 0, 255)

            if posture == "PRONE":
                alert = True

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"Worker: {posture} ({conf:.2f})"
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    return frame, alert
