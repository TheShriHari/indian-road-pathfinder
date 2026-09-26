#!/usr/bin/env python3
import sys
import os
import shutil
import subprocess
from pathlib import Path

def find_ffmpeg():
    # 1. PATH
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    # 2. Local AppData / winget paths
    user_profile = os.environ.get("USERPROFILE", "")
    local_app_data = os.environ.get("LOCALAPPDATA", "")
    candidates = [
        Path(local_app_data) / "Microsoft" / "WinGet" / "Links" / "ffmpeg.exe",
        Path(user_profile) / "scoop" / "shims" / "ffmpeg.exe",
        Path(user_profile) / "AppData" / "Local" / "Microsoft" / "WinGet" / "Packages",
    ]
    for c in candidates:
        if c.is_file():
            return str(c)
        if c.is_dir():
            found = list(c.glob("**/ffmpeg.exe"))
            if found:
                return str(found[0])
    # 3. compositor bundled
    bundled = Path(__file__).resolve().parents[4] / "sih-demo-video" / "node_modules" / "@remotion" / "compositor-win32-x64-msvc" / "ffmpeg.exe"
    if bundled.is_file():
        return str(bundled)
    return "ffmpeg"

def extract_with_ffmpeg(ffmpeg_bin, v, out, fps):
    # Extract timestamped keyframes
    cmd = [
        ffmpeg_bin, "-y", "-i", str(v),
        "-vf", f"fps={fps},scale=1280:-1,drawtext=text='%{{pts\\:hms}}':x=20:y=20:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6",
        "-q:v", "3", str(out / "frame_%04d.jpg")
    ]
    res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if res.returncode != 0:
        return False

    # Generate 4x4 contact sheet
    subprocess.run([
        ffmpeg_bin, "-y", "-i", str(v),
        "-vf", "fps=1/5,scale=480:-1,tile=4x4",
        "-q:v", "3", str(out / "montage.jpg")
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True

def extract_with_cv2(v, out, fps):
    import cv2
    import numpy as np

    cap = cv2.VideoCapture(str(v))
    if not cap.isOpened():
        print(f"[ERROR] Cannot open {v}")
        sys.exit(1)

    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, int(video_fps / fps))

    frame_idx = 0
    saved_count = 0
    montage_frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % step == 0:
            saved_count += 1
            sec = frame_idx / video_fps
            h = int(sec // 3600)
            m = int((sec % 3600) // 60)
            s = int(sec % 60)
            timestamp_str = f"{h:02d}:{m:02d}:{s:02d}"

            # Resize to width 1280
            h_orig, w_orig = frame.shape[:2]
            target_w = 1280
            target_h = int(h_orig * (target_w / w_orig))
            resized = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)

            # Draw timestamp badge
            cv2.rectangle(resized, (15, 10), (160, 50), (0, 0, 0), -1)
            cv2.putText(resized, timestamp_str, (25, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

            out_path = out / f"frame_{saved_count:04d}.jpg"
            cv2.imwrite(str(out_path), resized)

            # Sample every 5s for 4x4 montage
            if int(sec) % 5 == 0 and len(montage_frames) < 16:
                m_resized = cv2.resize(frame, (480, int(h_orig * (480 / w_orig))), interpolation=cv2.INTER_AREA)
                cv2.rectangle(m_resized, (10, 8), (110, 32), (0, 0, 0), -1)
                cv2.putText(m_resized, timestamp_str, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                montage_frames.append(m_resized)

        frame_idx += 1

    cap.release()

    # Create 4x4 montage
    if montage_frames:
        target_h, target_w = montage_frames[0].shape[:2]
        while len(montage_frames) < 16:
            montage_frames.append(np.zeros((target_h, target_w, 3), dtype=np.uint8))
        rows = []
        for r in range(4):
            row_tiles = montage_frames[r * 4 : (r + 1) * 4]
            rows.append(np.hstack(row_tiles))
        montage = np.vstack(rows)
        cv2.imwrite(str(out / "montage.jpg"), montage)

def extract(video_path, out_dir, fps=1.0):
    v = Path(video_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if not v.exists():
        print(f"[ERROR] Video does not exist: {v}")
        sys.exit(1)

    ffmpeg_bin = find_ffmpeg()
    success = False
    try:
        success = extract_with_ffmpeg(ffmpeg_bin, v, out, fps)
    except Exception:
        success = False

    if not success:
        print("[INFO] Falling back to OpenCV video frame extraction")
        extract_with_cv2(v, out, fps)

    print(f"[OK] Extracted review artifacts to: {out}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: extract_frames.py <video_path> <out_dir> [fps]")
        sys.exit(1)
    extract(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 1.0)
