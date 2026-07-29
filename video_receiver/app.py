"""Local MaixCAM JPEG receiver, recorder and playback server."""

from __future__ import annotations

import json
import re
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


ROOT = Path(__file__).resolve().parent
WEB_ROOT = ROOT / "web"
RECORDINGS = ROOT / "recordings"
MAX_FRAME_BYTES = 5 * 1024 * 1024
MAX_RECORDING_BYTES = 300 * 1024 * 1024

latest_frame = b""
latest_frame_at = 0.0
frame_lock = threading.Condition()


def json_bytes(payload: object) -> bytes:
    return json.dumps(payload, ensure_ascii=False).encode("utf-8")


class ReceiverHandler(SimpleHTTPRequestHandler):
    server_version = "MaixCAMReceiver/1.0"

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.log_date_time_string()}  {fmt % args}")

    def send_json(self, payload: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json_bytes(payload)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def read_body(self, maximum: int) -> bytes | None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_json({"error": "无效的内容长度"}, HTTPStatus.BAD_REQUEST)
            return None
        if length <= 0 or length > maximum:
            self.send_json({"error": "文件大小不符合要求"}, HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
            return None
        return self.rfile.read(length)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/frame":
            self.receive_frame()
            return
        if path == "/api/recordings/upload":
            self.save_recording()
            return
        self.send_json({"error": "接口不存在"}, HTTPStatus.NOT_FOUND)

    def receive_frame(self) -> None:
        body = self.read_body(MAX_FRAME_BYTES)
        if body is None:
            return
        if not body.startswith(b"\xff\xd8"):
            self.send_json({"error": "只接受 JPEG 图像"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
            return
        global latest_frame, latest_frame_at
        with frame_lock:
            latest_frame = body
            latest_frame_at = time.time()
            frame_lock.notify_all()
        self.send_json({"ok": True})

    def save_recording(self) -> None:
        body = self.read_body(MAX_RECORDING_BYTES)
        if body is None:
            return
        if not body.startswith(b"\x1aE\xdf\xa3"):
            self.send_json({"error": "录制文件格式无效"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
            return
        RECORDINGS.mkdir(exist_ok=True)
        name = datetime.now().strftime("recording_%Y%m%d_%H%M%S.webm")
        path = RECORDINGS / name
        path.write_bytes(body)
        self.send_json({"ok": True, "name": name})

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/status":
            age = time.time() - latest_frame_at if latest_frame_at else None
            self.send_json({
                "connected": age is not None and age < 3,
                "last_frame_age": round(age, 2) if age is not None else None,
            })
            return
        if path == "/api/recordings":
            self.list_recordings()
            return
        if path == "/api/frame.jpg":
            self.serve_latest_frame()
            return
        if path == "/stream.mjpg":
            self.stream_mjpeg()
            return
        if path.startswith("/recordings/"):
            self.serve_recording(path)
            return
        if path == "/":
            self.path = "/index.html"
        self.directory = str(WEB_ROOT)
        super().do_GET()

    def list_recordings(self) -> None:
        RECORDINGS.mkdir(exist_ok=True)
        entries = []
        for item in sorted(RECORDINGS.glob("*.webm"), key=lambda p: p.stat().st_mtime, reverse=True):
            stat = item.stat()
            entries.append({
                "name": item.name,
                "size": stat.st_size,
                "created": datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
            })
        self.send_json(entries)

    def serve_latest_frame(self) -> None:
        with frame_lock:
            frame = latest_frame
        if not frame:
            self.send_response(HTTPStatus.NO_CONTENT)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame)

    def serve_recording(self, request_path: str) -> None:
        name = unquote(request_path.removeprefix("/recordings/"))
        if not re.fullmatch(r"recording_\d{8}_\d{6}\.webm", name):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        file_path = RECORDINGS / name
        if not file_path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "video/webm")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def stream_mjpeg(self) -> None:
        boundary = "maixframe"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_sent_at = 0.0
        try:
            while True:
                with frame_lock:
                    frame_lock.wait_for(lambda: latest_frame_at > last_sent_at, timeout=2)
                    frame = latest_frame
                    sent_at = latest_frame_at
                if not frame or sent_at <= last_sent_at:
                    continue
                header = (
                    f"--{boundary}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n"
                ).encode("ascii")
                self.wfile.write(header)
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                last_sent_at = sent_at
        except (BrokenPipeError, ConnectionResetError):
            return


def main() -> None:
    RECORDINGS.mkdir(exist_ok=True)
    server = ThreadingHTTPServer(("0.0.0.0", 8000), ReceiverHandler)
    print("MaixCAM 接收端已启动")
    print("本机访问地址  http://127.0.0.1:8000")
    print("局域网访问地址  http://本机局域网地址:8000")
    print("按 Ctrl+C 停止服务")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务已停止")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
