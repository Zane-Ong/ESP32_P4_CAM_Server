#!/usr/bin/env python3
"""Convert the project's MJPEG recordings into MP4.

The recorder writes a fixed-size 256-byte text header at the start of the file.
This script reads that header, extracts the recorded FPS, removes the header,
and then asks ffmpeg to encode the remaining MJPEG stream into MP4.

Usage:
    python mjpeg_to_mp4.py input.mjpeg
    python mjpeg_to_mp4.py input.mjpeg -o output.mp4
    python mjpeg_to_mp4.py
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import filedialog
except Exception:  # pragma: no cover - GUI is optional
    tk = None
    filedialog = None

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - optional fallback
    cv2 = None
    np = None


HEADER_SIZE = 256


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert ESP32-P4 MJPEG recordings to MP4")
    parser.add_argument("input", nargs="?", type=Path, help="Input .mjpeg/.mjpg file")
    parser.add_argument("-o", "--output", type=Path, help="Output .mp4 path")
    parser.add_argument("--ffmpeg", default="ffmpeg", help="Path to ffmpeg executable")
    return parser.parse_args()


def choose_input_file() -> Path | None:
    if tk is None or filedialog is None:
        print("Tkinter is not available, please pass an input file path on the command line.", file=sys.stderr)
        return None

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    selected = filedialog.askopenfilename(
        title="Select MJPEG recording",
        filetypes=[("MJPEG recordings", "*.mjpeg *.mjpg"), ("All files", "*.*")],
    )
    root.destroy()

    if not selected:
        return None
    return Path(selected)


def maybe_reexec_with_venv() -> bool:
    if os.environ.get("MJPEG_TO_MP4_REEXEC") == "1":
        return False

    if cv2 is not None and np is not None:
        return False

    venv_python = Path(__file__).resolve().parent.parent / ".venv" / "Scripts" / "python.exe"
    if not venv_python.exists():
        return False

    cmd = [str(venv_python), str(Path(__file__).resolve()), *sys.argv[1:]]
    print("Current Python lacks OpenCV; restarting with workspace virtual environment...", file=sys.stderr)
    env = os.environ.copy()
    env["MJPEG_TO_MP4_REEXEC"] = "1"
    subprocess.run(cmd, check=True, env=env)
    return True


def read_header(input_path: Path) -> tuple[str, float | None]:
    with input_path.open("rb") as f:
        header = f.read(HEADER_SIZE)

    text = header.decode("ascii", errors="ignore").replace("\x00", "")
    fps_match = re.search(r"AVG_FPS=(\d+)\.(\d+)", text)
    fps = None
    if fps_match:
        fps = int(fps_match.group(1)) + int(fps_match.group(2)) / 1000.0

    return text, fps


def strip_header(input_path: Path, temp_dir: Path) -> Path:
    payload_path = temp_dir / f"{input_path.stem}.payload.mjpeg"
    with input_path.open("rb") as src, payload_path.open("wb") as dst:
        src.seek(HEADER_SIZE)
        shutil.copyfileobj(src, dst)
    return payload_path


def run_ffmpeg(ffmpeg: str, payload_path: Path, output_path: Path, fps: float) -> None:
    cmd = [
        ffmpeg,
        "-y",
        "-f",
        "mjpeg",
        "-framerate",
        f"{fps:.3f}",
        "-i",
        str(payload_path),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]

    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def extract_jpeg_frames(payload: bytes) -> list[bytes]:
    frames: list[bytes] = []
    offset = 0

    while True:
        start = payload.find(b"\xff\xd8", offset)
        if start < 0:
            break
        end = payload.find(b"\xff\xd9", start + 2)
        if end < 0:
            break
        frames.append(payload[start : end + 2])
        offset = end + 2

    return frames


def run_opencv(payload_path: Path, output_path: Path, fps: float) -> None:
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV is not available")

    payload = payload_path.read_bytes()
    frames = extract_jpeg_frames(payload)
    if not frames:
        raise RuntimeError("No JPEG frames found in recording")

    first_frame = cv2.imdecode(np.frombuffer(frames[0], dtype=np.uint8), cv2.IMREAD_COLOR)
    if first_frame is None:
        raise RuntimeError("Failed to decode first frame")

    height, width = first_frame.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("Failed to open MP4 writer")

    try:
        for index, frame_bytes in enumerate(frames, start=1):
            image = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                print(f"Skip undecodable frame #{index}", file=sys.stderr)
                continue
            if image.shape[1] != width or image.shape[0] != height:
                image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            writer.write(image)
    finally:
        writer.release()


def main() -> int:
    args = parse_args()

    if maybe_reexec_with_venv():
        return 0

    input_path = args.input or choose_input_file()
    if input_path is None:
        print("No input file selected.", file=sys.stderr)
        return 1

    if not input_path.exists():
        print(f"Input file not found: {input_path}", file=sys.stderr)
        return 1

    if args.output:
        output_path = args.output
    else:
        output_path = input_path.with_suffix(".mp4")

    header_text, fps = read_header(input_path)
    if fps is None or fps <= 0:
        print("Could not read AVG_FPS from header, falling back to 15.000 fps", file=sys.stderr)
        fps = 15.0

    print("Detected header:\n" + header_text.strip())
    print(f"Using FPS: {fps:.3f}")

    with tempfile.TemporaryDirectory(prefix="mjpeg2mp4_") as temp_dir_name:
        temp_dir = Path(temp_dir_name)
        payload_path = strip_header(input_path, temp_dir)
        if shutil.which(args.ffmpeg) is not None:
            run_ffmpeg(args.ffmpeg, payload_path, output_path, fps)
        else:
            print("ffmpeg not found; using OpenCV fallback", file=sys.stderr)
            run_opencv(payload_path, output_path, fps)

    print(f"Saved MP4 to: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())