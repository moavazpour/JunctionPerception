"""Run the trained 3D-keypoint model over a video and write an annotated copy (calibration-free view).

Each road user is drawn as its predicted 3D box (9 keypoints: bottom centre, footprint, top), front edge
highlighted, with class, confidence and a track id. Inference uses the training resolution and the same
letterboxing as the runtime.

    python -m yolo3d.render_video --weights best.pt --video IMG_5538.MOV --out IMG_5538_jp.mp4 --imgsz 960
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

COLORS = {"car": (255, 160, 60), "bus": (60, 200, 255), "truck": (60, 255, 160), "trailer": (200, 255, 60),
          "pedestrian": (80, 80, 255), "bicycle": (255, 80, 255), "escooter": (255, 255, 80), "motorcycle": (160, 80, 255)}
EDGES = [(1, 2), (2, 3), (3, 4), (4, 1), (5, 6), (6, 7), (7, 8), (8, 5), (1, 5), (2, 6), (3, 7), (4, 8)]
FRONT = [(1, 2), (5, 6), (1, 5), (2, 6)]


def draw(img, boxes, kpts, names, kp_conf, show_ids):
    for i in range(len(boxes)):
        cls = names[int(boxes.cls[i])]
        col = COLORS.get(cls, (255, 255, 255))
        k = kpts[i]
        ok = k[:, 2] >= kp_conf
        p = k[:, :2].astype(int)
        for a, b in EDGES:
            if ok[a] and ok[b]:
                front = (a, b) in FRONT
                cv2.line(img, tuple(p[a]), tuple(p[b]), (0, 0, 255) if front else col, 3 if front else 2, cv2.LINE_AA)
        if ok[0]:
            cv2.circle(img, tuple(p[0]), 4, (255, 255, 255), -1, cv2.LINE_AA)
        x1, y1 = boxes.xyxy[i][:2].cpu().numpy().astype(int)
        tid = int(boxes.id[i]) if show_ids and boxes.id is not None else None
        label = f"{cls} {float(boxes.conf[i]):.2f}" + (f" #{tid}" if tid is not None else "")
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        y0 = max(th + 4, y1 - 2)
        cv2.rectangle(img, (x1, y0 - th - 4), (x1 + tw + 4, y0), col, -1)
        cv2.putText(img, label, (x1 + 2, y0 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--imgsz", type=int, default=960, help="the training resolution")
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--kp_conf", type=float, default=0.35, help="draw a corner only above this confidence")
    ap.add_argument("--no_track", action="store_true")
    ap.add_argument("--max_frames", type=int, default=0)
    a = ap.parse_args()
    model = YOLO(a.weights)
    names = model.names
    cap = cv2.VideoCapture(str(a.video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w, h = int(cap.get(3)), int(cap.get(4))
    tmp = a.out.with_suffix(".tmp.mp4")
    vw = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok or (a.max_frames and n >= a.max_frames):
            break
        kw = dict(imgsz=a.imgsz, conf=a.conf, half=True, max_det=300, verbose=False)
        r = (model.predict(frame, **kw) if a.no_track else model.track(frame, persist=True, tracker="bytetrack.yaml", **kw))[0]
        if len(r.boxes) and r.keypoints is not None:
            draw(frame, r.boxes, r.keypoints.data.cpu().numpy(), names, a.kp_conf, not a.no_track)
        vw.write(frame)
        n += 1
        if n % 300 == 0:
            print(f"{n} frames", flush=True)
    vw.release()
    # Re-encode to H.264 so the file plays everywhere (browsers, phones); keep mp4v if no encoder is available.
    ff = shutil.which("ffmpeg")
    if ff is None:
        try:
            import imageio_ffmpeg

            ff = imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:
            ff = None
    if ff:
        subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(tmp), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                        "-movflags", "+faststart", str(a.out)], check=True)
        tmp.unlink()
    else:
        tmp.rename(a.out)
    print(f"wrote {n} frames ({w}x{h} @ {fps:.1f} fps) -> {a.out}")


if __name__ == "__main__":
    main()
