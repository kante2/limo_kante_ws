#!/usr/bin/env python3
# cam_view.py
import cv2
import time
import argparse
from pathlib import Path

def open_cam(device, width=None, height=None, fps=None):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)  # V4L2로 강제 오픈(리눅스 권장)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {device}")

    # 원하는 해상도/FPS가 있으면 시도
    # if width  is not None: cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(width))
    # if height is not None: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    # if fps    is not None: cap.set(cv2.CAP_PROP_FPS,          int(fps))

    # 포맷(선택): MJPG가 더 빠른 경우가 많음
    # fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    # cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    # 적용 결과 로그
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    f = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Opened {device} → {w}x{h} @ {f:.1f} FPS")
    return cap

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0",
                    help="카메라 장치 경로 또는 인덱스(예: 0)")
    ap.add_argument("--width",  type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps",    type=int, default=30)
    args = ap.parse_args()

    # 숫자 인덱스도 허용 (예: --device 0)
    device = int(args.device) if str(args.device).isdigit() else args.device

    cap = open_cam(device, args.width, args.height, args.fps)

    cv2.namedWindow("LIMO Camera (/dev/video0)", cv2.WINDOW_NORMAL)
    prev = time.time()
    cnt  = 0
    smoothed_fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Failed to grab frame")
            break

        # FPS 측정(이동평균)
        cnt += 1
        now = time.time()
        if cnt >= 10:
            dt = now - prev
            if dt > 0:
                instant = cnt / dt
                smoothed_fps = 0.9 * smoothed_fps + 0.1 * instant if smoothed_fps else instant
            prev, cnt = now, 0

        # 오버레이
        txt = f"{frame.shape[1]}x{frame.shape[0]}  FPS:{smoothed_fps:4.1f}"
        cv2.putText(frame, txt, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2, cv2.LINE_AA)

        cv2.imshow("LIMO Camera (/dev/video0)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC or q
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
