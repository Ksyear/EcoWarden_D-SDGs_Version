#!/usr/bin/env python3
"""
FastAPI 쓰레기 투기 이벤트 전송 테스트
POST https://api.ecowarden.systems/api/dumping-event

사용법:
    export ECOWARDEN_API_KEY=<발급받은 키>
    python3 test_fastapi.py              # dumping 이벤트 테스트
    python3 test_fastapi.py intrusion    # 금지구역 침입 이벤트 테스트
"""

import os
import sys
import requests
import time
import json

URL = os.environ.get(
    "ECOWARDEN_API_URL", "https://api.ecowarden.systems/api/dumping-event")
INTRUSION_URL = os.environ.get(
    "ECOWARDEN_INTRUSION_URL", URL.replace("dumping-event", "intrusion-event"))

# 키는 반드시 환경변수로 주입한다. 하드코딩 금지 (노출된 레거시 키는 서버에서 회전됨).
API_KEY = os.environ.get("ECOWARDEN_API_KEY", "")
if not API_KEY:
    sys.exit("ECOWARDEN_API_KEY 환경변수를 설정하세요. "
             "예: export ECOWARDEN_API_KEY=<발급받은 키>")

HEADERS = {
    "Content-Type": "application/json",
    "X-API-Key": API_KEY,
}


def post(url, payload):
    print("=== 전송할 JSON ===")
    print(json.dumps(payload, indent=2))
    print(f"=== POST {url} ===\n")

    try:
        resp = requests.post(url, headers=HEADERS, json=payload, timeout=10)
        print(f"Status: {resp.status_code}")
        print(f"Response: {resp.text}")
        return resp.status_code
    except requests.exceptions.ConnectionError as e:
        print(f"연결 실패: {e}")
        return None
    except requests.exceptions.Timeout:
        print("타임아웃 (10초)")
        return None


def send_dumping_event():
    now_ns = int(time.time() * 1_000_000_000)

    payload = {
        "person_id": 1,
        "person_x": 1.200,
        "person_y": 0.350,
        "cumulative_dist": 3.456,
        "object_id": 2,
        "object_x": 0.800,
        "object_y": 0.200,
        "event_time": now_ns,
        # v50: 발생 존(0~4)과 금지구역 여부. 서버가 무시해도 무방한 추가 필드.
        "zone": 3,
        "severity": "high",
        # "image_base64": ""  # 이미지 없이 테스트
    }
    return post(URL, payload)


def send_intrusion_event():
    now_ns = int(time.time() * 1_000_000_000)

    payload = {
        "type": "intrusion",
        "person_id": 1,
        "person_x": 1.200,
        "person_y": 0.350,
        "zone": 3,
        "event_time": now_ns,
    }
    return post(INTRUSION_URL, payload)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "intrusion":
        status = send_intrusion_event()
    else:
        status = send_dumping_event()

    if status and 200 <= status < 300:
        print("\n성공!")
    else:
        print("\n실패 - 서버 상태 확인 필요")
