#!/usr/bin/env python3
import socket
import json
import pygame
import sys
import math

# ── 설정 (UDP 포트 및 화면 크기) ───────────────────────────────────
UDP_IP = "0.0.0.0"
UDP_PORT = 9090
WIDTH, HEIGHT = 800, 800
SCALE = 0.1  # 1mm -> 0.1px (8m 범위 표시)

# ── 색상 ──────────────────────────────────────────────
COLOR_BG = (10, 20, 10)
COLOR_GRID = (20, 60, 20)
COLOR_POINT = (50, 255, 50)
COLOR_PERSON = (255, 100, 100)
COLOR_OBJECT = (255, 255, 100)
COLOR_TEXT = (150, 200, 150)

def main():
    # Pygame 초기화
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("EcoWarden LiDAR Visualizer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 15)

    # UDP 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    data = {"points": [], "objects": [], "tracks": []}

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # UDP 데이터 수신 (논블로킹)
        try:
            while True:
                packet, addr = sock.recvfrom(65535)
                new_data = json.loads(packet.decode('utf-8'))
                if new_data.get("type") == "FRAME":
                    data = new_data
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Error: {e}")

        # 그리기 시작
        screen.fill(COLOR_BG)
        center = (WIDTH // 2, HEIGHT // 2)

        # 1. 레이더 그리드 그리기
        for r in range(1000, 8001, 1000):
            pygame.draw.circle(screen, COLOR_GRID, center, int(r * SCALE), 1)
        
        pygame.draw.line(screen, COLOR_GRID, (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), 1)
        pygame.draw.line(screen, COLOR_GRID, (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), 1)

        # 2. 원시 점구름 그리기 (샘플링됨)
        for p in data.get("points", []):
            px = int(center[0] + p[0] * SCALE)
            py = int(center[1] - p[1] * SCALE)
            if 0 <= px < WIDTH and 0 <= py < HEIGHT:
                pygame.draw.circle(screen, COLOR_POINT, (px, py), 2)

        # 3. 객체 및 트랙 그리기
        for obj in data.get("objects", []):
            ox = int(center[0] + obj["x"] * SCALE)
            oy = int(center[1] - obj["y"] * SCALE)
            color = COLOR_OBJECT if obj["type"] != "normal" else COLOR_GRID
            pygame.draw.rect(screen, color, (ox - 5, oy - 5, 10, 10), 2)

        for tr in data.get("tracks", []):
            tx = int(center[0] + tr["x"] * SCALE)
            ty = int(center[1] - tr["y"] * SCALE)
            color = COLOR_PERSON if not tr["is_dumped_item"] else COLOR_OBJECT
            
            # 상태 표시
            pygame.draw.circle(screen, color, (tx, ty), 8, 2)
            lbl = font.render(f"ID:{tr['id']} {tr['state']}", True, color)
            screen.blit(lbl, (tx + 10, ty - 10))

        # 4. 정보 텍스트
        info = font.render(f"Frame: {data.get('frame_id', 0)} | Pts: {len(data.get('points', []))}", True, COLOR_TEXT)
        screen.blit(info, (10, 10))
        
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()
