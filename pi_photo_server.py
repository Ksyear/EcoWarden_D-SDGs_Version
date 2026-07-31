#!/usr/bin/env python3
"""
라즈베리파이의 증거 사진을 Unity 로 바로 넘겨주는 최소 HTTP 서버.

백엔드(FastAPI)가 없어도 Unity 가 사진을 받을 수 있게 하는 우회 경로다.
임베디드 C++ 코드는 건드리지 않는다 — rplidar_app 이 이미 captures/ 아래에
JPEG 를 저장하고 있으므로, 그것을 읽어 서빙하기만 한다.

원본: Unity 팀 제공 (2026-07-31). 엔드포인트와 응답 형식은 그대로 유지하고
임베디드 쪽에서 아래 세 가지만 보강했다.

  [보안] /file/ 에 확장자·디렉터리 화이트리스트 추가
      원본은 `_serve_file()` 에 확장자 검사가 없어서 captures/ 아래 **아무
      파일이나** 나갔다. v56 부터 captures/vault/ 에는 마스킹 전 원본의
      암호문(.ewv)과 **열람 로그(access.log)** 가 쌓인다. 즉
      `GET /file/vault/access.log` 로 "누가 언제 무슨 사유로 증거를 열었는지"
      가 통째로 유출될 수 있었다. 이미지 확장자만 서빙하고 vault/ 는 차단한다.

  [보안] 선택적 토큰 인증
      LAN 에 붙은 누구나 증거 사진을 가져갈 수 있는 상태였다. 시연망이라
      기본은 꺼두되, --token 을 주면 X-Auth-Token 헤더나 ?token= 을 요구한다.

  [성능] /latest.jpg 디렉터리 스캔 캐시
      원본은 요청마다 captures/ 전체를 os.walk 했다. Unity 가 폴링하면
      촬영이 쌓일수록 느려진다. 1초 캐시를 둔다 (사진은 그보다 자주 안 생김).

실행 — rplidar_app 과 같은 작업 디렉터리에서:

    python3 pi_photo_server.py                 # 기본 포트 8088, ./captures 감시
    python3 pi_photo_server.py --port 9000 --root /home/pi/ecowarden/captures
    python3 pi_photo_server.py --token <임의문자열>   # 인증 켜기

엔드포인트:
    GET /file/<captures 기준 상대경로>   image_file 로 지목된 정확한 사진
    GET /latest.jpg   가장 최근 이미지 (image_file 이 없을 때 폴백)
    GET /health       상태 확인용 JSON
    GET /list         최근 20개 파일 목록 JSON
"""
import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse, parse_qs

IMAGE_EXTS = (".jpg", ".jpeg", ".png")

# captures/ 아래에서 **절대 서빙하면 안 되는** 디렉터리.
#   vault/ — 마스킹 전 원본 암호문(.ewv) + 열람 로그(access.log)
DENY_DIRS = ("vault",)

_scan_lock = threading.Lock()
_scan_cache = {"at": 0.0, "root": None, "images": []}
_SCAN_TTL_SEC = 1.0


def _is_denied(rel_path):
    """captures/ 기준 상대경로가 차단 디렉터리 안에 있는가."""
    parts = os.path.normpath(rel_path).split(os.sep)
    return any(p in DENY_DIRS for p in parts)


def find_images(root, use_cache=True):
    """captures/ 아래 이미지를 (수정시각, 경로) 로 모아 최신순 정렬.

    vault/ 는 제외한다. 결과는 짧게 캐시해 폴링 부하를 줄인다.
    """
    now = time.time()
    with _scan_lock:
        if (use_cache and _scan_cache["root"] == root
                and now - _scan_cache["at"] < _SCAN_TTL_SEC):
            return list(_scan_cache["images"])

    found = []
    root_abs = os.path.abspath(root)
    for dirpath, dirnames, filenames in os.walk(root):
        # 차단 디렉터리는 내려가지 않는다
        dirnames[:] = [d for d in dirnames if d not in DENY_DIRS]
        for name in filenames:
            if not name.lower().endswith(IMAGE_EXTS):
                continue
            path = os.path.join(dirpath, name)
            try:
                found.append((os.path.getmtime(path), path))
            except OSError:
                pass  # 스캔 도중 지워진 파일은 무시
    found.sort(reverse=True)

    with _scan_lock:
        _scan_cache.update({"at": now, "root": root, "images": list(found)})
    return found


class Handler(BaseHTTPRequestHandler):
    root = "captures"
    token = None          # None 이면 인증 없음

    # ── 응답 헬퍼 ────────────────────────────────────────────────────
    def _send(self, code, body, ctype="application/json", extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # Unity 가 같은 URL 을 반복 호출하므로 캐시를 막는다
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self, query):
        if not self.token:
            return True
        supplied = (self.headers.get("X-Auth-Token")
                    or (query.get("token") or [None])[0])
        return supplied == self.token

    def _serve_image(self, abspath):
        """이미지 확장자만 서빙한다. 호출 전에 경로 검증을 마쳐야 한다."""
        if not abspath.lower().endswith(IMAGE_EXTS):
            self._send(403, json.dumps({"error": "not an image"}).encode())
            return
        try:
            with open(abspath, "rb") as f:
                data = f.read()
            mtime = os.path.getmtime(abspath)
        except OSError as e:
            self._send(404, json.dumps({"error": str(e)}).encode())
            return
        ctype = "image/png" if abspath.lower().endswith(".png") else "image/jpeg"
        self._send(200, data, ctype, extra={
            "X-Photo-Name": os.path.basename(abspath),
            "X-Photo-Age-Sec": f"{time.time() - mtime:.1f}",
        })
        print(f"[serve] {abspath}  ({len(data)} bytes, {time.time()-mtime:.1f}초 전)")

    # ── 라우팅 ───────────────────────────────────────────────────────
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if not self._authorized(query):
            self._send(401, json.dumps({"error": "unauthorized"}).encode())
            return

        # /file/<captures 기준 상대경로> — image_file 로 정확히 지목된 사진.
        if path.startswith("/file/"):
            rel = unquote(path[len("/file/"):])
            root_abs = os.path.abspath(self.root)
            target = os.path.abspath(os.path.join(root_abs, rel))

            # 상위 디렉터리 탈출(../) 차단
            if os.path.commonpath([root_abs, target]) != root_abs:
                self._send(403, json.dumps({"error": "forbidden path"}).encode())
                return
            # vault/ 등 차단 디렉터리
            if _is_denied(os.path.relpath(target, root_abs)):
                self._send(403, json.dumps(
                    {"error": "forbidden directory"}).encode())
                print(f"[deny] 차단 디렉터리 접근 시도: {rel}")
                return
            if not os.path.isfile(target):
                self._send(404, json.dumps(
                    {"error": "not found", "path": rel}).encode())
                return
            self._serve_image(target)
            return

        if path in ("/latest.jpg", "/latest"):
            images = find_images(self.root)
            if not images:
                self._send(404, json.dumps({"error": "no images"}).encode())
                return
            self._serve_image(images[0][1])
            return

        if path == "/health":
            images = find_images(self.root)
            body = {
                "ok": True,
                "root": os.path.abspath(self.root),
                "image_count": len(images),
                "auth": bool(self.token),
            }
            if images:
                body["newest"] = os.path.basename(images[0][1])
                body["newest_age_sec"] = round(time.time() - images[0][0], 1)
            self._send(200, json.dumps(body, ensure_ascii=False).encode())
            return

        if path == "/list":
            images = find_images(self.root)[:20]
            body = [{"name": os.path.basename(p),
                     "path": os.path.relpath(p, os.path.abspath(self.root)),
                     "age_sec": round(time.time() - m, 1)} for m, p in images]
            self._send(200, json.dumps(body, ensure_ascii=False).encode())
            return

        self._send(404, json.dumps({"error": "not found"}).encode())

    def log_message(self, *args):
        pass  # 기본 액세스 로그는 끄고 위의 [serve]/[deny] 만 남긴다


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--root", default="captures")
    ap.add_argument("--token", default=os.environ.get("ECOWARDEN_PHOTO_TOKEN"),
                    help="설정 시 X-Auth-Token 헤더 또는 ?token= 요구")
    args = ap.parse_args()

    if not os.path.isdir(args.root):
        print(f"경고: '{args.root}' 디렉터리가 없습니다. "
              f"rplidar_app 과 같은 위치에서 실행했는지 확인하세요.")

    Handler.root = args.root
    Handler.token = args.token
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"증거 사진 서버 시작 — http://0.0.0.0:{args.port}")
    print(f"  감시 디렉터리 : {os.path.abspath(args.root)}")
    print(f"  최신 사진     : http://<파이IP>:{args.port}/latest.jpg")
    print(f"  상태 확인     : http://<파이IP>:{args.port}/health")
    print(f"  인증          : {'켜짐' if args.token else '꺼짐 (LAN 전체 공개)'}")
    print(f"  차단 디렉터리 : {', '.join(DENY_DIRS)} (암호화 원본·열람 로그)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n종료")


if __name__ == "__main__":
    main()
