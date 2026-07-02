#!/usr/bin/env python3
"""ARES 收图服务：接收 ESP32-CAM 上传的照片，存到 vision/incoming/。

用法:
    python receive_server.py [--port 8000] [--dir incoming]

ESP32-CAM 固件会 POST 到 http://<本机IP>:8000/upload?name=<照片名>
（PC_HOST/PC_PORT 在 firmware/cam/config.h 里配置）
只用标准库，无需安装任何依赖。
"""
import argparse
import re
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

SAVE_DIR = Path(__file__).parent / "incoming"
SAFE_NAME = re.compile(r"[^A-Za-z0-9_\-]")


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        n = len(list(SAVE_DIR.glob("*.jpg")))
        self._reply(200, f"ARES receive_server OK, {n} photos in {SAVE_DIR}\n")

    def do_POST(self):
        url = urlparse(self.path)
        if url.path != "/upload":
            self._reply(404, "unknown path\n")
            return
        name = parse_qs(url.query).get("name", ["unnamed"])[0]
        name = SAFE_NAME.sub("_", name)[:80] or "unnamed"
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0 or length > 8 * 1024 * 1024:
            self._reply(400, "bad length\n")
            return
        data = self.rfile.read(length)
        path = SAVE_DIR / f"{name}.jpg"
        if path.exists():  # 不覆盖：加时间戳
            path = SAVE_DIR / f"{name}_{datetime.now():%H%M%S}.jpg"
        path.write_bytes(data)
        print(f"[{datetime.now():%H:%M:%S}] 收到 {path.name}  ({length/1024:.0f} KB)")
        self._reply(200, "OK\n")

    def _reply(self, code, text):
        body = text.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):  # 静默默认访问日志
        pass


def main():
    global SAVE_DIR
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--dir", default=str(SAVE_DIR), help="照片保存目录")
    args = ap.parse_args()
    SAVE_DIR = Path(args.dir)
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    print(f"收图服务已启动: http://0.0.0.0:{args.port}/upload  ->  {SAVE_DIR}")
    print("提示: firmware/cam/config.h 的 PC_HOST 要填本机在热点网络中的 IP")
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
