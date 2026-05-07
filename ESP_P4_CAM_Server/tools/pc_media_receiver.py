#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import struct
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterator
from urllib.parse import urlparse


LOG = logging.getLogger("pc_media_receiver")


def utc_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def sanitize_filename(name: str, fallback: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in ("-", "_", ".") else "_" for ch in name).strip("._")
    return cleaned or fallback


def find_jpeg_size(frame: bytes) -> tuple[int, int]:
    i = 2
    size = len(frame)
    while i + 9 < size:
        if frame[i] != 0xFF:
            i += 1
            continue
        marker = frame[i + 1]
        i += 2
        if marker in (0xD8, 0xD9):
            continue
        if i + 2 > size:
            break
        segment_len = struct.unpack(">H", frame[i:i + 2])[0]
        if segment_len < 2 or i + segment_len > size:
            break
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC9, 0xCA, 0xCB):
            height = struct.unpack(">H", frame[i + 3:i + 5])[0]
            width = struct.unpack(">H", frame[i + 5:i + 7])[0]
            return width, height
        i += segment_len
    raise ValueError("unable to parse jpeg frame size")


class MjpegAviWriter:
    def __init__(self, path: Path, width: int, height: int, fps: int) -> None:
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        self.max_frame_size = 0
        self.index: list[tuple[int, int]] = []
        self.file = path.open("wb")
        self._write_placeholder_headers()

    def _write_placeholder_headers(self) -> None:
        f = self.file
        f.write(b"RIFF")
        self.riff_size_pos = f.tell()
        f.write(struct.pack("<I", 0))
        f.write(b"AVI ")

        f.write(b"LIST")
        self.hdrl_size_pos = f.tell()
        f.write(struct.pack("<I", 0))
        f.write(b"hdrl")

        f.write(b"avih")
        f.write(struct.pack("<I", 56))
        self.avih_pos = f.tell()
        f.write(b"\x00" * 56)

        f.write(b"LIST")
        self.strl_size_pos = f.tell()
        f.write(struct.pack("<I", 0))
        f.write(b"strl")

        f.write(b"strh")
        f.write(struct.pack("<I", 56))
        self.strh_pos = f.tell()
        f.write(b"\x00" * 56)

        f.write(b"strf")
        f.write(struct.pack("<I", 40))
        self.strf_pos = f.tell()
        f.write(b"\x00" * 40)

        self.hdrl_end = f.tell()

        f.write(b"LIST")
        self.movi_size_pos = f.tell()
        f.write(struct.pack("<I", 0))
        f.write(b"movi")
        self.movi_data_start = f.tell()

    def add_frame(self, jpeg_frame: bytes) -> None:
        chunk_start = self.file.tell()
        self.file.write(b"00dc")
        self.file.write(struct.pack("<I", len(jpeg_frame)))
        self.file.write(jpeg_frame)
        if len(jpeg_frame) % 2:
            self.file.write(b"\x00")

        self.index.append((chunk_start - self.movi_data_start, len(jpeg_frame)))
        self.frame_count += 1
        self.max_frame_size = max(self.max_frame_size, len(jpeg_frame))

    def close(self) -> None:
        if self.file.closed:
            return

        movi_end = self.file.tell()
        self.file.write(b"idx1")
        self.file.write(struct.pack("<I", len(self.index) * 16))
        for offset, size in self.index:
            self.file.write(struct.pack("<4sIII", b"00dc", 0x10, offset, size))

        file_end = self.file.tell()
        microseconds_per_frame = int(1_000_000 / max(self.fps, 1))
        max_bytes_per_sec = self.max_frame_size * max(self.fps, 1)

        self.file.seek(self.avih_pos)
        self.file.write(struct.pack(
            "<IIIIIIIIII4I",
            microseconds_per_frame,
            max_bytes_per_sec,
            0,
            0x910,
            self.frame_count,
            0,
            1,
            self.max_frame_size,
            self.width,
            self.height,
            0, 0, 0, 0,
        ))

        self.file.seek(self.strh_pos)
        self.file.write(struct.pack(
            "<4s4sIHHIIIIIIIIhhhh",
            b"vids",
            b"MJPG",
            0,
            0,
            0,
            0,
            1,
            self.fps,
            0,
            self.frame_count,
            self.max_frame_size,
            0xFFFFFFFF,
            0,
            0, 0, self.width, self.height,
        ))

        self.file.seek(self.strf_pos)
        self.file.write(struct.pack(
            "<IiiHH4sIiiII",
            40,
            self.width,
            self.height,
            1,
            24,
            b"MJPG",
            self.max_frame_size,
            0,
            0,
            0,
            0,
        ))

        self.file.seek(self.hdrl_size_pos)
        self.file.write(struct.pack("<I", self.hdrl_end - (self.hdrl_size_pos + 4)))

        self.file.seek(self.strl_size_pos)
        self.file.write(struct.pack("<I", self.hdrl_end - (self.strl_size_pos + 4)))

        self.file.seek(self.movi_size_pos)
        self.file.write(struct.pack("<I", movi_end - (self.movi_size_pos + 4)))

        self.file.seek(self.riff_size_pos)
        self.file.write(struct.pack("<I", file_end - 8))

        self.file.close()


def iter_mjpeg_frames(stream_url: str, duration_s: int) -> Iterator[bytes]:
    req = urllib.request.Request(stream_url, headers={"User-Agent": "ESP32-P4-PC-Recorder"})
    deadline = time.monotonic() + duration_s
    buffer = b""

    with urllib.request.urlopen(req, timeout=15) as response:
        while time.monotonic() < deadline:
            chunk = response.read(4096)
            if not chunk:
                break
            buffer += chunk

            while True:
                soi = buffer.find(b"\xff\xd8")
                if soi < 0:
                    buffer = buffer[-1:] if buffer else b""
                    break

                eoi = buffer.find(b"\xff\xd9", soi + 2)
                if eoi < 0:
                    if soi > 0:
                        buffer = buffer[soi:]
                    break

                frame = buffer[soi:eoi + 2]
                buffer = buffer[eoi + 2:]
                yield frame


def record_stream_to_avi(stream_url: str, duration_s: int, output_dir: Path, fps_hint: int = 30) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    avi_path = output_dir / f"record_{utc_stamp()}.avi"
    writer: MjpegAviWriter | None = None
    frame_counter = 0

    try:
        for frame in iter_mjpeg_frames(stream_url, duration_s):
            if writer is None:
                width, height = find_jpeg_size(frame)
                writer = MjpegAviWriter(avi_path, width, height, fps_hint)
                LOG.info("recording %s at %dx%d", avi_path.name, width, height)

            writer.add_frame(frame)
            frame_counter += 1
    finally:
        if writer is not None:
            writer.close()

    if frame_counter == 0:
        if avi_path.exists():
            avi_path.unlink()
        raise RuntimeError("no frames received from mjpeg stream")

    LOG.info("record finished: %s, frames=%d", avi_path, frame_counter)
    return avi_path


class MediaReceiverServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], handler_class: type[BaseHTTPRequestHandler], output_dir: Path) -> None:
        super().__init__(server_address, handler_class)
        self.output_dir = output_dir


class MediaRequestHandler(BaseHTTPRequestHandler):
    server: MediaReceiverServer

    def _json_response(self, status: HTTPStatus, payload: dict) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self) -> None:
        if self.path == "/health":
            self._json_response(HTTPStatus.OK, {"status": "ok"})
            return
        self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        content_length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(content_length)

        if parsed.path == "/api/photo":
            self._handle_photo(body)
            return
        if parsed.path == "/api/record":
            self._handle_record(body)
            return

        self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def _handle_photo(self, body: bytes) -> None:
        photos_dir = self.server.output_dir / "photos"
        photos_dir.mkdir(parents=True, exist_ok=True)
        file_name = sanitize_filename(self.headers.get("X-Filename", f"capture_{utc_stamp()}.jpg"), f"capture_{utc_stamp()}.jpg")
        out_path = photos_dir / file_name
        out_path.write_bytes(body)
        LOG.info("saved photo: %s", out_path)
        self._json_response(HTTPStatus.OK, {"saved": str(out_path)})

    def _handle_record(self, body: bytes) -> None:
        try:
            payload = json.loads(body.decode("utf-8"))
            stream_url = payload["stream_url"]
            duration_s = int(payload.get("duration_s", 15))

            LOG.info(f"RECEIVED STREAM_URL: {stream_url}") #2026/4/6n

        except (KeyError, ValueError, json.JSONDecodeError) as exc:
            self._json_response(HTTPStatus.BAD_REQUEST, {"error": f"invalid payload: {exc}"})
            return

        records_dir = self.server.output_dir / "records"

        def worker() -> None:
            try:
                record_stream_to_avi(stream_url, duration_s, records_dir)
            except urllib.error.URLError as exc:
                LOG.error("record request failed: %s", exc)
            except Exception as exc:  # noqa: BLE001
                LOG.exception("record worker failed: %s", exc)

        threading.Thread(target=worker, daemon=True).start()
        LOG.info("accepted record request: %s for %ds", stream_url, duration_s)
        self._json_response(HTTPStatus.ACCEPTED, {"accepted": True})

    def log_message(self, fmt: str, *args: object) -> None:
        LOG.info("%s - %s", self.address_string(), fmt % args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive ESP32-P4 photos and recordings on a PC")
    parser.add_argument("--host", default="0.0.0.0", help="bind host, default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=8088, help="bind port, default: 8088")
    parser.add_argument("--output-dir", default="pc_receiver_output", help="output directory")
    parser.add_argument("--log-level", default="INFO", help="logging level")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(message)s")
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    server = MediaReceiverServer((args.host, args.port), MediaRequestHandler, output_dir)
    LOG.info("pc media receiver listening on http://%s:%d", args.host, args.port)
    LOG.info("output directory: %s", output_dir)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        LOG.info("receiver stopped")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
