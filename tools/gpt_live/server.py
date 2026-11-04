#!/usr/bin/env python3
"""Local GPT-Live WebRTC client. Python standard library only."""
import argparse
import json
import os
from pathlib import Path
import re
import sqlite3
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent


def credentials():
    key = os.environ.get("OPENAI_API_KEY")
    if key:
        return key, os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1")
    db = Path.home() / ".cc-switch/cc-switch.db"
    if db.exists():
        with sqlite3.connect(db.as_uri() + "?mode=ro", uri=True) as connection:
            for (raw,) in connection.execute(
                "SELECT settings_config FROM providers WHERE app_type='codex' AND name='OpenAI API'"
            ):
                config = json.loads(raw)
                key = config.get("auth", {}).get("OPENAI_API_KEY")
                # Only reuse the official OpenAI provider, never forward its key elsewhere.
                base = re.search(r'^base_url\s*=\s*"([^"]+)"', config.get("config", ""), re.M)
                if key and base and base[1].rstrip("/") == "https://api.openai.com/v1":
                    return key, base[1]
    return "", "https://api.openai.com/v1"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def reply(self, status, value, content_type="application/json; charset=utf-8"):
        body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def allowed_host(self):
        return self.headers.get("Host") in self.server.hosts

    def do_GET(self):
        if not self.allowed_host():
            return self.reply(403, {"error": "不允许的主机"})
        files = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/app.js": ("app.js", "text/javascript; charset=utf-8")}
        if self.path not in files:
            return self.reply(404, {"error": "未找到页面"})
        name, kind = files[self.path]
        self.reply(200, (ROOT / name).read_bytes(), kind)

    def do_POST(self):
        if not self.allowed_host() or self.headers.get("Origin") not in self.server.origins:
            return self.reply(403, {"error": "只接受本地页面请求"})
        if self.path != "/api/session":
            return self.reply(404, {"error": "未知接口"})
        if self.headers.get_content_type() != "application/json" or self.headers.get("Transfer-Encoding"):
            return self.reply(400, {"error": "需要 JSON 请求"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 65536:
                return self.reply(413, {"error": "请求大小必须在 1–65536 字节之间"})
            offer = json.loads(self.rfile.read(length))
            sdp = offer.get("sdp") if isinstance(offer, dict) else None
            if not isinstance(sdp, str) or not sdp.startswith("v=0"):
                raise ValueError()
        except (ValueError, UnicodeError):
            return self.reply(400, {"error": "无效的 SDP offer"})
        entered_key = offer.get("api_key", "")
        if not isinstance(entered_key, str) or (entered_key and not re.fullmatch(r"[!-~]{1,512}", entered_key)):
            return self.reply(400, {"error": "密钥格式无效，请检查空格或换行。"})
        api_key = entered_key or self.server.api_key
        if not api_key:
            return self.reply(400, {"error": "请先在页面填写 OpenAI API 密钥。"})
        # Page-entered keys go only to OpenAI, independent of the server gateway.
        base_url = "https://api.openai.com/v1" if entered_key else self.server.base_url
        payload = {
            "session": {
                "model": "gpt-live-1",
                "instructions": "请用中文自然、简洁地交流。需要复杂推理时委派后台处理。",
                "delegation": {"type": "responses", "responses": {
                    "model": os.environ.get("OPENAI_BACKEND_MODEL", "gpt-5.6-terra"),
                    "instructions": "请用中文准确简洁地回答。你没有本地文件或机器人控制权限。",
                }},
            },
            "transport": {"type": "webrtc", "sdp": sdp},
        }
        request = Request(base_url.rstrip("/") + "/live/sessions",
                          data=json.dumps(payload).encode(), headers={
                              "Authorization": "Bearer " + api_key,
                              "Content-Type": "application/json",
                          }, method="POST")
        try:
            with urlopen(request, timeout=30) as response:
                result = json.load(response)
            if not (isinstance(result.get("session", {}).get("id"), str)
                    and isinstance(result.get("transport", {}).get("sdp"), str)):
                raise ValueError()
            self.reply(201, {"session": {"id": result["session"]["id"]},
                             "transport": {"sdp": result["transport"]["sdp"]}})
        except HTTPError as error:
            # Never reflect upstream bodies: gateways may echo credentials.
            self.reply(error.code, {"error": f"OpenAI 返回 HTTP {error.code}；请检查余额、模型权限及 Live 接口支持。"})
        except (URLError, TimeoutError, OSError):
            self.reply(502, {"error": "连接 OpenAI 失败或超时，请检查网络。"})
        except (ValueError, AttributeError, TypeError):
            self.reply(502, {"error": "OpenAI 返回了无效的会话信息。"})


def make_server(port, key, base_url):
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    port = server.server_port
    server.api_key, server.base_url = key, base_url
    server.hosts = {f"localhost:{port}", f"127.0.0.1:{port}"}
    server.origins = {"http://" + host for host in server.hosts}
    return server


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--open", action="store_true", help="启动后自动打开浏览器")
    args = parser.parse_args()
    try:
        key, base_url = credentials()
    except (ValueError, sqlite3.Error):
        print("无法读取本机默认密钥，请在页面填写。", flush=True)
        key, base_url = "", "https://api.openai.com/v1"
    with make_server(args.port, key, base_url) as server:
        print(f"打开 http://localhost:{server.server_port} ，按 Ctrl+C 停止。", flush=True)
        if args.open:
            webbrowser.open(f"http://localhost:{server.server_port}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
