#!/usr/bin/env python3
"""EcoWarden 보안 감시 시스템 — LiDAR 실시간 시각화 + 수신 데이터 값 표시.

장치가 UDP(기본 127.0.0.1:9090)로 보내는 FRAME JSON을 그대로 받아
레이더 화면과 함께 "실제로 어떤 값이 전송되는지"를 우측 패널에 띄운다.
(Unity 5005 포트로 가는 tracked_only 패킷도 이 포트로 받으면 동일하게 표시 가능)

키:
  d — 데이터 패널 표시/숨김
  c — 이벤트 로그 지우기
  q / ESC — 종료
"""
import socket
import json
import pygame
import math
import time
from collections import deque

# ── 설정 (UDP 포트 및 화면 크기) ───────────────────────────────────
UDP_IP = "0.0.0.0"
UDP_PORT = 9090
RADAR_W, HEIGHT = 800, 800
PANEL_W = 400                     # 우측 데이터 패널 폭
SCALE = 0.1                       # 1mm -> 0.1px (8m 범위 표시)
INTRUSION_FLASH_SEC = 5.0         # 침입 존 강조 유지 시간

# ── 색상 ──────────────────────────────────────────────
COLOR_BG = (10, 20, 10)
COLOR_PANEL_BG = (16, 16, 24)
COLOR_GRID = (20, 60, 20)
COLOR_POINT = (50, 255, 50)
COLOR_PERSON = (255, 100, 100)
COLOR_OBJECT = (255, 255, 100)
COLOR_TEXT = (150, 200, 150)
COLOR_HEADER = (120, 180, 255)
COLOR_INTRUSION = (255, 60, 60)
COLOR_DUMPING = (255, 200, 60)
COLOR_DEPARTURE = (130, 130, 130)

EVENT_COLORS = {
    "intrusion": COLOR_INTRUSION,
    "dumping": COLOR_DUMPING,
    "departure": COLOR_DEPARTURE,
}


def zone_dir(angle_deg):
    """존 각도(0~180°) → 화면 방향 벡터. 장치의 SuspectToZone과 동일 기준:
    angle = atan2(x, |y|) + 90° 이므로 x = sin(a-90°), y = cos(a-90°)."""
    a = math.radians(angle_deg - 90.0)
    return math.sin(a), math.cos(a)


def format_event(evt):
    """이벤트 1건을 '전송된 값 그대로' 한 줄 텍스트로 변환."""
    t = evt.get("type", "?")
    if t == "intrusion":
        return (f"INTRUSION person={evt.get('person_track_id')} "
                f"zone={evt.get('zone')} sev={evt.get('severity')} "
                f"({evt.get('person_x'):.0f},{evt.get('person_y'):.0f}) "
                f"ts={evt.get('timestamp')}")
    if t == "dumping":
        return (f"DUMPING person={evt.get('person_track_id')} "
                f"obj={evt.get('object_track_id')} "
                f"({evt.get('object_x'):.0f},{evt.get('object_y'):.0f}) "
                f"ts={evt.get('timestamp')}")
    if t == "departure":
        return (f"DEPARTURE id={evt.get('object_track_id')} "
                f"({evt.get('object_x'):.0f},{evt.get('object_y'):.0f}) "
                f"ts={evt.get('timestamp')}")
    return json.dumps(evt, ensure_ascii=False)


def event_key(evt):
    """intrusion/departure는 UDP 손실 대비 5프레임 반복 전송되므로
    (type, id, timestamp)로 중복 수신을 걸러낸다."""
    return (evt.get("type"),
            evt.get("person_track_id"),
            evt.get("object_track_id"),
            evt.get("timestamp"))


def main():
    pygame.init()
    screen = pygame.display.set_mode((RADAR_W + PANEL_W, HEIGHT))
    pygame.display.set_caption("EcoWarden Security Surveillance Visualizer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 15)
    font_s = pygame.font.SysFont("monospace", 13)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    data = {"points": [], "objects": [], "tracks": [], "events": []}
    show_panel = True
    seen_events = set()            # 반복 전송 dedupe
    event_log = deque(maxlen=200)  # (수신시각, 이벤트 dict)
    intrusion_zones = {}           # zone → 강조 만료 시각

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_d:
                    show_panel = not show_panel
                elif event.key == pygame.K_c:
                    event_log.clear()
                    seen_events.clear()

        # UDP 데이터 수신 (논블로킹)
        try:
            while True:
                packet, addr = sock.recvfrom(65535)
                new_data = json.loads(packet.decode("utf-8"))
                if new_data.get("type") != "FRAME":
                    continue
                data = new_data
                for evt in new_data.get("events", []):
                    key = event_key(evt)
                    if key in seen_events:
                        continue
                    seen_events.add(key)
                    event_log.append((time.strftime("%H:%M:%S"), evt))
                    # 콘솔에도 전송 값 원본을 남긴다 (기록/디버그용)
                    print(f"[EVENT] {json.dumps(evt, ensure_ascii=False)}")
                    if evt.get("type") == "intrusion":
                        zone = evt.get("zone")
                        if zone is not None:
                            intrusion_zones[zone] = time.time() + INTRUSION_FLASH_SEC
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Error: {e}")

        # 그리기 시작
        screen.fill(COLOR_BG)
        center = (RADAR_W // 2, HEIGHT // 2)

        # 1. 레이더 그리드
        for r in range(1000, 8001, 1000):
            pygame.draw.circle(screen, COLOR_GRID, center, int(r * SCALE), 1)
        pygame.draw.line(screen, COLOR_GRID, (0, HEIGHT // 2), (RADAR_W, HEIGHT // 2), 1)
        pygame.draw.line(screen, COLOR_GRID, (RADAR_W // 2, 0), (RADAR_W // 2, HEIGHT), 1)

        # 1.5 침입 존 강조 (intrusion 이벤트의 zone 값을 부채꼴로 표시)
        now = time.time()
        intrusion_zones = {z: exp for z, exp in intrusion_zones.items() if exp > now}
        for zone in intrusion_zones:
            wedge = [center]
            for a in range(int(zone * 45 - 22), int(zone * 45 + 23), 3):
                dx, dy = zone_dir(max(0, min(180, a)))
                wedge.append((center[0] + dx * 8000 * SCALE,
                              center[1] - dy * 8000 * SCALE))
            surf = pygame.Surface((RADAR_W, HEIGHT), pygame.SRCALPHA)
            pygame.draw.polygon(surf, (255, 60, 60, 50), wedge)
            screen.blit(surf, (0, 0))

        # 2. 원시 점구름 (샘플링됨)
        for p in data.get("points", []):
            px = int(center[0] + p[0] * SCALE)
            py = int(center[1] - p[1] * SCALE)
            if 0 <= px < RADAR_W and 0 <= py < HEIGHT:
                pygame.draw.circle(screen, COLOR_POINT, (px, py), 2)

        # 3. 객체 및 트랙
        for obj in data.get("objects", []):
            ox = int(center[0] + obj["x"] * SCALE)
            oy = int(center[1] - obj["y"] * SCALE)
            color = COLOR_OBJECT if obj["type"] != "normal" else COLOR_GRID
            pygame.draw.rect(screen, color, (ox - 5, oy - 5, 10, 10), 2)

        for tr in data.get("tracks", []):
            tx = int(center[0] + tr["x"] * SCALE)
            ty = int(center[1] - tr["y"] * SCALE)
            color = COLOR_PERSON if not tr["is_dumped_item"] else COLOR_OBJECT
            pygame.draw.circle(screen, color, (tx, ty), 8, 2)
            lbl = font.render(f"ID:{tr['id']} {tr['state']}", True, color)
            screen.blit(lbl, (tx + 10, ty - 10))

        # 4. 상단 정보 + 침입 경보 배너
        info = font.render(
            f"Frame: {data.get('frame_id', 0)} | Pts: {len(data.get('points', []))} "
            f"| Obj: {len(data.get('objects', []))} | Trk: {len(data.get('tracks', []))} "
            f"| [d] data panel", True, COLOR_TEXT)
        screen.blit(info, (10, 10))
        if intrusion_zones:
            banner = font.render(
                f"!! INTRUSION zone={sorted(intrusion_zones.keys())} !!",
                True, COLOR_INTRUSION)
            screen.blit(banner, (10, 30))

        # 5. 우측 데이터 패널 — 장치가 전송한 값 원본
        if show_panel:
            pygame.draw.rect(screen, COLOR_PANEL_BG, (RADAR_W, 0, PANEL_W, HEIGHT))
            pygame.draw.line(screen, COLOR_GRID, (RADAR_W, 0), (RADAR_W, HEIGHT), 2)
            x0, y = RADAR_W + 10, 10

            def line(text, color=COLOR_TEXT, small=True, indent=0):
                nonlocal y
                f = font_s if small else font
                screen.blit(f.render(text[:52], True, color), (x0 + indent, y))
                y += 16 if small else 18

            line("== 수신 데이터 (device -> UDP JSON) ==", COLOR_HEADER, small=False)
            line(f"type=FRAME frame_id={data.get('frame_id', 0)}")
            line(f"timestamp={data.get('timestamp', 0)} (epoch ms)")
            y += 6

            objs = data.get("objects", [])
            line(f"-- objects[{len(objs)}] (mm) --", COLOR_HEADER)
            for obj in objs[:8]:
                line(f"id={obj.get('id')} {obj.get('type')} "
                     f"x={obj.get('x'):.0f} y={obj.get('y'):.0f} "
                     f"w={obj.get('width', 0):.0f} "
                     f"aband={obj.get('is_abandoned')}", indent=6)
            if len(objs) > 8:
                line(f"... 외 {len(objs) - 8}건", indent=6)
            y += 6

            trks = data.get("tracks", [])
            line(f"-- tracks[{len(trks)}] --", COLOR_HEADER)
            for tr in trks[:8]:
                flags = []
                if tr.get("is_dump_suspect"):
                    flags.append("suspect")
                if tr.get("is_dumped_item"):
                    flags.append("dumped")
                if tr.get("person_group_id"):
                    flags.append(f"grp={tr['person_group_id']}")
                line(f"id={tr.get('id')} {tr.get('state')} "
                     f"x={tr.get('x'):.0f} y={tr.get('y'):.0f} "
                     f"{' '.join(flags)}", indent=6)
            if len(trks) > 8:
                line(f"... 외 {len(trks) - 8}건", indent=6)
            y += 6

            line(f"-- events 로그 (신규 {len(event_log)}건, [c] clear) --", COLOR_HEADER)
            for ts, evt in list(event_log)[-14:]:
                color = EVENT_COLORS.get(evt.get("type"), COLOR_TEXT)
                line(f"{ts} {format_event(evt)}", color, indent=2)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
