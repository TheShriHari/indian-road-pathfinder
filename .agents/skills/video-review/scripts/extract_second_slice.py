#!/usr/bin/env python3
import sys
import os
import subprocess
import shutil
from pathlib import Path

def find_ffmpeg():
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
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
    return "ffmpeg"

def extract_second(video_path: str, sec: int, out_dir: str = ".tmp/second_audit"):
    out = Path(out_dir)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    ffmpeg_bin = find_ffmpeg()
    cmd = [
        ffmpeg_bin, "-y", "-ss", str(sec), "-t", "1",
        "-i", str(video_path),
        "-vf", "fps=10,scale=1280:-1,drawtext=text='T\\=%{pts\\:hms} (F\\=%{n})':x=20:y=20:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6",
        "-q:v", "2", str(out / "frame_%02d.jpg")
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    frames = sorted(list(out.glob("frame_*.jpg")))
    print(f"[OK] Extracted {len(frames)} frames for Second {sec:02d} -> {out}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python extract_second_slice.py <video_path> <second_int> [out_dir]")
        sys.exit(1)
    extract_second(sys.argv[1], int(sys.argv[2]), sys.argv[3] if len(sys.argv) > 3 else ".tmp/second_audit")
