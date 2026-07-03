# EcoWarden: 데이터 무결성 보증형 디지털 트윈 관제 플랫폼 (Embedded)

RPLiDAR S2와 USB 야간 카메라를 이용한 불법 투기 실시간 탐지 및 증거 보존 시스템의 임베디드 모듈입니다. 10Hz 이상의 고속 LiDAR 스캔과 비동기 카메라 프레임 버퍼링을 결합하여 투기 발생 즉시 증거를 확보하고 서버로 전송합니다.

탐지는 **지능형 다중 센서 융합 알고리즘**(DBSCAN 클러스터링 + 칼만 필터 추적 + 다단계 투기 판정 FSM) 기반입니다. 학습형 신경망 모델은 임베디드에 탑재하지 않으며, 확정 증거 사진이 서버로 전송되므로 서버 측에서 이미지 분류 AI로 2차 검증을 붙이는 확장이 가능합니다.

**현재 버전**: v50 — 금지구역(존) 침입 탐지 + 블랙박스 전후 10초 영상 + 증거 사진 ID/존/시각 오버레이 (자세한 변경 이력은 §9 참조)

---

## 1. 하드웨어 사양 (Hardware Info)

| 구분            | 상세 모델 및 규격                                                         | 용도                                    |
| :------------ | :----------------------------------------------------------------- | :------------------------------------ |
| **LiDAR**     | Slamtec RPLiDAR S2 (ToF, 30m)                                      | 실시간 거리 측정, 객체 탐지                      |
| **Camera**    | [DFROBOT] 2MP USB Night Vision [FIT0730]                           | 비동기 30fps 증거 캡처 (1080p, IR 지원)        |
| **Servo**     | SG90급 3선 hobby servo (기본: Arduino USB serial / Pi5 단독: GPIO18 PWM) | suspect 좌표 기준 카메라 5-zone pan 조준       |
| **Sensor**    | Panasonic PaPIRs (EKMC160111) x 2                                  | 정밀 디지털 모션 감지 (좌: GPIO 17, 우: GPIO 27) |
| **Computing** | Raspberry Pi 5 + Ubuntu Linux                                      | 핵심 로직 및 통신 처리                         |

---

## 2. 시스템 아키텍처 (Architecture)

### 2.1 데이터 흐름 파이프라인

```
Hardware Layer
├─ RplidarS2 (USB 3.0, 10Hz @ 1Mbps)
├─ CameraModule (async 30fps MJPEG + 블랙박스 링버퍼 전후 10초)
├─ ServoController (Arduino serial 기본 | Pi5 GPIO18 PWM fallback, 5-zone pan, worker thread)
└─ PirSensor (GPIO 17/27, libgpiod + sysfs fallback)
         │
Processing Layer (Main Loop, 10Hz target)
├─ ScanProcessor
│  ├─ FilterNoise (distance/intensity/FOV gate)
│  ├─ FilterStaticBackground (confidence + neighbor edge guard)
│  ├─ ToCartesian (polar → x/y mm)
│  ├─ DBSCAN (ε=100mm, min_pts=3, grid O(n))
│  └─ LearnStaticBackground (EMA alpha=0.02, tracked-object 슬롯 + edge guard 보호)
│
├─ BackgroundFilter
│  ├─ LearnFrame (50-frame warmup, 30% threshold)
│  ├─ FilterBackground (radius: 80mm)
│  └─ 학습 중에도 시각화 데이터 전송 (점구름 + 빈 트랙)
│
└─ ClusterTracker (7-Phase Dumping FSM)
   ├─ Phase 0.5: KF Predict (전 트랙 1회)
   ├─ Phase 1: AssociateGreedy (2-pass + KF fallback)
   ├─ Phase 1.5: DetectClusterSplit
   ├─ Phase 2: 매칭 트랙 갱신 (KF Update, 속도, 폭, 궤적, confirmed 승격)
   ├─ Phase 3: 미매칭 트랙 → lost 카운트
   ├─ Phase 4: suspect → dumped_item 승격 (PersonGroup 기준 사람 이탈 검증)
   ├─ Phase 5: lost/tentative 트랙 삭제 + 위치 버퍼 관리
   ├─ Phase 6: pending cluster quarantine → DetectSeparation → Tentative 새 트랙
   ├─ Phase 6.5: 기존 정지 트랙 궤적 매칭 재검사
   └─ Phase 7: CheckDumpingConfirmation
         │
├─ ZonePolicy (금지구역 정책 — ECOWARDEN_RESTRICTED_ZONES 설정 시)
│  └─ 사람 트랙의 존이 금지 존에 연속 N프레임 관측되면 intrusion 이벤트
         │
Output Layer
├─ JsonPacketSender → Unity UDP (JSON, 기본 192.168.20.173:5005)
├─ JsonPacketSender → 시각화기 UDP (JSON, 127.0.0.1:9090)
├─ UdpSender       → Unity 바이너리 프로토콜 (옵션, ECOWARDEN_UNITY_BINARY=1)
├─ EventNotifier   → HTTP POST dumping-event / intrusion-event (X-API-Key)
├─ ServoController → suspect/person 좌표를 5-zone으로 변환해 카메라 조준/저장
└─ CameraModule    → JPEG capture (ID/존/시각 오버레이) + 블랙박스 AVI (전후 10초)
```

### 2.2 Source of Truth (데이터 무결성의 근간)

| 관심사 | 단일 출처 | 파일 |
| :--- | :--- | :--- |
| **운영 파라미터** | `production_params.h` | `Default*Params()` 4종 inline 함수 |
| **시간 유틸리티** | `time_utils.h` | `NowMs()`, `MsToIso8601()`, `MsToApiTime()` |
| **칼만 필터 수학** | `kalman_filter.h` | header-only, Joseph form 수치 안정 |
| **트랙 상태 머신** | `cluster_tracker.h` | `TrackState` enum + `Track` struct |
| **프레임 프로파일** | `phase_profiler.h` | 10-phase CSV export |

### 2.3 신호 노이즈 전파 차단 구조

```
Raw LiDAR (noisy)
  ↓ [Gate 1] FilterNoise: distance ∈ [150, 12000]mm, intensity ≥ 10, FOV ∈ [0°, 180°]
  ↓ [Gate 2] FilterStaticBackground: confidence ≥ 10 + neighbor guard + 150mm margin
  ↓ [Gate 3] DBSCAN: ε=100mm, min_pts=3 → NOISE 레이블 제거
  ↓ [Gate 4] Width filter: ∈ [50, 1200]mm
  ↓ [Gate 5] MergeClusters: centroid ≤ 350mm + width ratio ≤ 2.5
  ↓ [Gate 6] Association: greedy NN ≤ 400mm + KF fallback capped at 500mm
  ↓ [Gate 7] Track confidence: Tentative → Confirmed (4 consecutive matches)
  ↓ [Gate 8] PersonGroup: anti-phase 보행을 허용하며 leg track을 사람 1명으로 그룹화
  ↓ [Gate 9] Output gate: Unity JSON/바이너리는 group 대표 synthetic object와 object/suspect/dumped만 송신
  ↓ [Gate 10] Pending cluster quarantine: unmatched cluster 1~3 frame 관측 후 공개
  ↓ [Gate 11] Separation: 궤적 600mm + 현재 200mm + lost recovery + hotspot + deleted pos
  ↓ [Gate 12] Suspect visible gate: source lock + birth gate + direct evidence + stability
  ↓ [Gate 13] Confirmation: source lock + birth gate + safe fast/strong confirm
Clean Detection (verified)
```

---

## 3. 실행 방법 (How to Run)

### 3.1 명령어 형식

```bash
./build/rplidar_app [시리얼포트] [FastAPI URL] [Unity IP:포트] [시각화 IP:포트]
```

### 3.2 인자 설명

| 순서 | 역할 | 기본값 | 비고 |
|------|------|--------|------|
| 1 | LiDAR 시리얼 포트 | `/dev/ttyUSB0` | |
| 2 | FastAPI 엔드포인트 | `https://api.ecowarden.systems/api/dumping-event` | |
| 3 | Unity 주소 | `192.168.20.173:5005` | 와이파이 변경 시 IP 수정 필요 |
| 4 | 시각화기 주소 | `127.0.0.1:9090` | localhost이므로 변경 불필요 |

### 3.3 실행 예시

```bash
# 기본값으로 실행 (Unity IP가 192.168.20.173일 때)
./build/rplidar_app

# Unity PC IP가 바뀌었을 때
./build/rplidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event 192.168.20.173:5005

# 시각화기 (별도 터미널)
python3 lidar_visualizer.py
```

### 3.4 네트워크 구조

```
rplidar_app ──UDP JSON──→ Unity PC (192.168.20.173:5005)   ← 와이파이 IP 의존
            ──UDP 바이너리→ Unity PC (ECOWARDEN_UNITY_BINARY=1일 때만)
            ──UDP JSON──→ localhost:9090 (시각화기)            ← 와이파이 무관
            ──HTTPS────→ api.ecowarden.systems (FastAPI)     ← DNS, 와이파이 무관
```

- **Unity UDP와 시각화기 UDP는 독립 소켓** — Unity가 꺼져 있어도 시각화기는 정상 동작
- UDP는 connectionless — 수신자가 없어도 송신측에 에러 없음

### 3.5 환경변수

| 변수 | 값 | 효과 |
|------|-----|------|
| `ECOWARDEN_DRY_RUN` | `1` / `true` | FastAPI 전송 비활성화 (테스트용) |
| `ECOWARDEN_LOG_LEVEL` | `frame` / `debug` | 프레임별 상세 로깅 활성화 |
| `ECOWARDEN_SERVO_ENABLE` | `0` / `1` | 서보 카메라 조준 활성화. PWM 실패 시 camera-only fallback |
| `ECOWARDEN_SERVO_MIRROR` | `0` / `1` | 서보 장착 방향 좌우 반전 |
| `ECOWARDEN_SERVO_BACKEND` | 기본 `arduino` | `arduino`는 USB serial로 각도 전송, `pwm`은 Pi sysfs PWM fallback |
| `ECOWARDEN_SERVO_SERIAL_DEVICE` | 기본 `/dev/ttyACM0` | Arduino USB serial 장치 |
| `ECOWARDEN_SERVO_SERIAL_BAUD` | 기본 `115200` | Arduino sketch와 맞추는 baud rate |
| `ECOWARDEN_SERVO_PWM_CHIP` / `ECOWARDEN_SERVO_PWM_CHANNEL` | `pwmchip2` / `0` | PWM fallback 전용 |
| `ECOWARDEN_SERVO_MIN_PULSE_US` / `ECOWARDEN_SERVO_MAX_PULSE_US` | `544` / `2400` | SG90 기준 0~180도 pulse 범위 |
| `ECOWARDEN_SERVO_SUSPECT_DIR` / `ECOWARDEN_SERVO_DUMP_DIR` | `captures/suspects` / `captures/dumps` | suspect 임시 사진과 확정 bundle 저장 위치 |
| `ECOWARDEN_SERVO_PERSON_DIR` | `captures/person_zones` | 사람 감지 zone 촬영 저장 위치 |
| `ECOWARDEN_SERVO_PERSON_CAPTURE_INTERVAL_MS` | `2000` | 같은 zone 반복 촬영 최소 간격 |
| `ECOWARDEN_SERVO_PERSON_SESSION_TIMEOUT_MS` | `3000` | 사람이 사라진 뒤 투기 없으면 임시 사진 삭제까지 기다리는 시간 |
| `ECOWARDEN_SERVO_ZONE_HYSTERESIS_DEG` | 기본 `10` | zone 경계 ±이 각도 이내에서는 현재 zone 유지 (경계 oscillation 방지, `0`=비활성) |
| `ECOWARDEN_SERVO_SLEW_STEP_DEG` / `ECOWARDEN_SERVO_SLEW_STEP_MS` | `15` / `20` | 서보 분할 이동 스텝 각도/간격 (오버슛 방지, `0`=단발 이동) |
| `ECOWARDEN_TENTATIVE_CONFIRM_FRAMES` | 기본 `4` | 신규 트랙을 Unity에 공개하기 전 연속 매칭 프레임 수. 저노이즈 시연 환경에서 줄이면 반응이 빨라짐 |
| `ECOWARDEN_RESTRICTED_ZONES` | 예: `3,4` | 금지구역(투기/출입 금지) 존 목록. 비어 있으면 침입 탐지 비활성 (§3.7) |
| `ECOWARDEN_INTRUSION_MIN_FRAMES` | 기본 `3` | 금지 존 연속 관측 프레임 수 — 존 경계 노이즈 오탐 방지 |
| `ECOWARDEN_INTRUSION_REPEAT_MS` | 기본 `10000` | 같은 사람 침입 재알림 최소 간격 |
| `ECOWARDEN_INTRUSION_URL` | (자동 유도) | intrusion 이벤트 endpoint. 미설정 시 dumping URL에서 `intrusion-event`로 치환 |
| `ECOWARDEN_INTRUSION_NOTIFY` | 기본 `1` | `0`이면 침입 이벤트 서버 전송만 끔 (촬영/블랙박스/로그는 유지) |
| `ECOWARDEN_BLACKBOX_ENABLE` | 기본 `1` | 이벤트 전후 영상 링버퍼 활성 (§3.8) |
| `ECOWARDEN_BLACKBOX_SEC` | 기본 `10` | 이벤트 전/후 저장 구간(초). `_PRE_SEC`/`_POST_SEC`로 개별 지정 가능 |
| `ECOWARDEN_BLACKBOX_FPS` | 기본 `10` | 링버퍼 프레임 레이트. 메모리와 트레이드오프 |
| `ECOWARDEN_API_KEY` | (비밀값) | FastAPI `X-API-Key`. 미설정 시 노출된 레거시 키로 fallback + 경고 로그 |
| `ECOWARDEN_QUEUE_FILE` | 기본 `/tmp/rplidar_event_queue.jsonl` | 전송 실패 이벤트 큐 파일 경로. 운영 배포는 `/var/lib/ecowarden/` 권장 |

#### 런타임 시그널

| 시그널 | 효과 |
|------|------|
| `SIGUSR1` | 정적 배경 + 배경 맵 즉시 리셋 후 재학습 (`kill -USR1 $(pgrep rplidar_app)`). 반복 촬영/센서 재배치 시 프로세스 재시작 불필요 |

### 3.6 서보 카메라 조준

기본 방식은 Arduino가 SG90 PWM을 만들고, Raspberry Pi는 LiDAR 좌표를 zone/angle로 바꿔 USB serial로 Arduino에 전송한다. Arduino에는 `arduino/servo_zone_controller/servo_zone_controller.ino`를 업로드한다.

| 연결 | 대상 |
|---|---|
| Raspberry Pi USB | Arduino USB |
| SG90 주황/노랑 Signal | Arduino D9 |
| SG90 빨강 VCC | Arduino 5V 또는 외부 5V `+` |
| SG90 갈색/검정 GND | Arduino GND. 외부 5V 사용 시 외부 전원 `-`와 공통 GND |

주의: 외부 5V 어댑터를 쓰면 **외부 GND와 Arduino GND를 반드시 묶어야** 한다. Arduino는 USB로 Pi와 연결되므로 Pi는 serial 명령만 보내고 PWM은 만들지 않는다.

벤치 테스트:

```bash
cmake --build build-codex --target servo_sweep_test
sudo ./build-codex/servo_sweep_test
```

운영 중에는 한 번 선택한 사람/사람그룹 ID를 계속 추적하고, 해당 좌표를 `0, 45, 90, 135, 180도` zone으로 변환해 카메라를 조준한다. 사진은 `captures/person_zones/person_<id>/`에 임시 보관한다. 같은 zone은 기본 2초 간격으로만 다시 찍고, zone이 바뀌면 즉시 새 사진을 남긴다. 사람이 사라진 뒤 기본 3초 동안 투기 확정이 없으면 해당 `person_<id>` 폴더를 삭제한다. 투기 확정 시에는 해당 사람 폴더를 `captures/dumps/<event>/person_<id>/`로 이동한다. 투기 suspect가 발생하면 person zone 촬영보다 suspect 촬영을 우선한다.

테스트 실행 중에는 `EcoWarden Servo Sweep` 창으로 카메라 live preview를 계속 보여주고, 서보가 `0, 45, 90, 135, 180도`로 이동할 때마다 `captures/sweep/zone_0.jpg`부터 `zone_4.jpg`까지 저장한다. 창에서 `q` 또는 `Esc`를 누르면 preview만 멈추고 진행 중인 sweep은 마무리한다.

좌우가 반대로 움직이면 코드 수정 대신 `ECOWARDEN_SERVO_MIRROR=1`로 실행한다.
기본 sweep 각도는 `0, 45, 90, 135, 180도`다.

zone 경계에서 LiDAR 각도 노이즈(±2~3°)로 서보가 두 zone을 오가지 않도록 ±10° hysteresis가 기본 적용되고, 전각도 이동 시 오버슛을 막기 위해 15°/스텝(20ms 간격) slew rate로 분할 이동한다. 둘 다 환경변수로 끄거나 조정할 수 있다.

LiDAR USB 연결이 끊겨 스캔이 5회 연속 실패하면 앱이 종료되지 않고 2초 간격으로 재연결을 시도하며, 재연결 후에도 학습된 배경 맵은 유지되어 재학습 대기가 없다.

증거 사진(사람 존 촬영·suspect·confirm)에는 좌상단에 `ID:<트랙ID> ZONE:<존> <UTC시각>` 배너가 렌더링된다. 사진이 단독으로 유통돼도 어떤 객체를 언제 어느 존에서 찍었는지 사진만으로 식별할 수 있다.

### 3.7 금지구역(존) 침입 탐지

"버리면 안 되는 지역"(보호구역·출입 제한 구역)을 존 단위로 지정하면, LiDAR가 추적하는 사람이 해당 존에 들어왔을 때 투기 여부와 무관하게 침입 이벤트를 만든다.

```bash
# 존 3, 4를 금지구역으로 지정 (존 번호는 §3.6의 5-zone과 동일: 0~4)
sudo env ECOWARDEN_RESTRICTED_ZONES=3,4 ./build/rplidar_app
```

침입 확정 시 동작:

1. 서보 카메라를 침입자 존으로 즉시 조준·촬영
2. FastAPI `intrusion-event`로 `person_id`, `zone`, 좌표, 시각, 사진(base64) 전송
3. `captures/intrusions/`에 블랙박스 영상(전후 10초) 저장
4. 해당 존에서 투기가 확정되면 dumping 이벤트의 `severity`가 `"high"`로 표기

오탐 방지: 존 경계 각도 노이즈로 1~2프레임 금지 존에 걸치는 경우를 걸러내기 위해 기본 3프레임 연속 관측을 요구하고(`ECOWARDEN_INTRUSION_MIN_FRAMES`), 같은 사람은 기본 10초 간격으로만 재알림한다(`ECOWARDEN_INTRUSION_REPEAT_MS`). 서버가 intrusion endpoint를 아직 지원하지 않으면 `ECOWARDEN_INTRUSION_NOTIFY=0`으로 전송만 끌 수 있다 (촬영·영상·로그는 유지).

### 3.8 블랙박스 증거 영상 (전후 10초)

카메라 캡처 스레드가 최근 프레임을 JPEG로 압축해 메모리 링버퍼에 상시 보관하다가, 투기 확정 또는 금지구역 침입 시 **이벤트 전 10초 + 후 10초** 구간을 AVI(MJPG)로 저장한다. 정지 사진만으로는 투기 순간의 행위 연속성을 입증하기 어렵다는 한계를 보완한다.

- 투기 확정: `captures/dumps/<event>/blackbox.avi` (suspect/confirm 사진, meta.json과 같은 번들)
- 금지구역 침입: `captures/intrusions/intrusion_<person>_<ts>.avi`
- 저장은 별도 worker thread에서 수행되어 10Hz scan loop를 막지 않는다
- 기본 10초/10fps/JPEG 75 기준 링버퍼 메모리는 1080p에서 약 40~60MB (Pi5 8GB 여유 범위)
- `ECOWARDEN_BLACKBOX_SEC`(전/후 공통), `_PRE_SEC`/`_POST_SEC`(개별), `_FPS`, `_ENABLE`로 조정

---

## 4. 핵심 알고리즘 (Core Algorithms)

### 4.1 칼만 필터 (2D Constant Velocity)

- **상태**: `[x, y, vx, vy]` (4D), **관측**: `[x, y]` (2D)
- **수치 안정**: Joseph form `(I-KH)P(I-KH)^T + KRK^T`
- **적응형 게이트**: `max(base_gate², 9 × PositionUncertaintySq)` — lost 시 P 성장으로 게이트 자동 확장
- **dt 파라미터**: `Predict(double dt)` — 프레임 지터 반영

### 4.2 DBSCAN 클러스터링

- **가속**: Grid-based spatial index → O(n) average (cell = ε × ε)
- **scratch 재사용**: `scratch_grid_`, `scratch_labels_`, `scratch_neighbors_` 멤버 영속화
- **소형 객체 대응**: `min_points=3`으로 15cm 높이에서 점 3~4개만 남는 투기물 후보 보존
- **후처리**: Union-Find 기반 다리 병합 (기본 350mm, width ratio 2.5 guard)
- **다리쌍 확장 병합**: 코드 지원은 있으나 운영 기본값은 비활성(`leg_pair_merge_radius_mm=0`). 현장 로그에서 보폭 분리가 남을 때만 450mm부터 실측 적용

### 4.3 정적 배경 학습 보호

- **EMA 속도**: `static_bg_ema_alpha=0.02`로 배경 적응을 느리게 해 정지한 소형 투기물이 너무 빨리 배경화되는 문제 완화
- **Edge guard**: `static_bg_min_confidence=10`, `static_bg_edge_guard_slots=1`, `static_bg_margin_mm=150`으로 단일 슬롯 edge/reflection spike를 DBSCAN 전에 제거
- **학습 차단 슬롯**: 활성 track 주변 각도 슬롯만 차단. 현재 프레임의 모든 클러스터를 무조건 차단하지 않음
- **이유**: 모든 클러스터를 보호하면 배경/정적 물체가 배경으로 재흡수되지 못하고 Tentative→Confirmed를 거쳐 사람 객체처럼 송신될 수 있음
- **효과**: 추적 중인 사람/투기 후보는 보호하면서, 트랙이 붙지 않은 정적 배경 클러스터는 다시 배경 모델로 편입 가능

### 4.4 PersonGroup / ObjectCandidate

- **PersonGroup**: 15cm 설치 환경에서 보폭 때문에 분리된 두 leg track을 하나의 사람 그룹으로 묶음. 보행 anti-phase 때문에 순간 속도 dot product가 음수여도 거리/폭 조건이 맞으면 group 허용
- **Group hold**: 보폭/속도 흔들림으로 group 조건이 1~5프레임 깨져도 직전 group을 유지해 1명/2명 깜박임을 줄임
- **Slow-walk pair**: 느린 보행 다리가 순간적으로 `object_candidate`처럼 보여도 최소 이동 이력 또는 기존 group 이력이 있으면 다리쌍 PersonGroup 후보로 허용
- **Wide-leg pair**: 650mm를 넘는 신규 쌍은 각 cluster 폭이 260mm 이하이고 생성 시점 차이가 6프레임 이내이며 주변에 제3 leg 후보가 없을 때만 1500mm까지 허용
- **Single-leg attach**: 기존 PersonEntity 주변 1600mm 안의 leg-like track은 아직 confirmed/moving이 아니어도 같은 사람 후보로 붙임. 직전 group 이력이 있는 held leg는 1800mm까지 원래 group에만 재attach하고, group 없는 `object_candidate`는 Unity JSON/바이너리 송신 차단
- **Wide-leg birth gap**: 실제 보폭에서 양쪽 다리 cluster 생성 시점이 늦게 갈라지는 경우를 위해 넓은 보폭 pair의 birth gap을 6프레임까지 허용하고 group hold를 18프레임으로 유지
- **Unity 송신**: JSON과 바이너리 모두 group 대표 synthetic object만 `normal` 객체로 송신해 사람 1명이 2명처럼 보이는 현상을 줄임
- **ObjectCandidate**: 정지한 소형 track은 사람 그룹에서 제외하고 `object_candidate` 타입으로 분리 유지
- **투기 확정**: source track이 PersonGroup에 속하면 group 중심을 기준으로 이탈 거리를 계산. 신규 물체 + source lock + 직접 분리 증거가 있으면 fast confirm 경로 사용
- **주의**: 신규 다리쌍 생성은 기본 650mm로 제한하되 좁은 leg pair는 Y축 깊이 차이가 500mm 이하일 때 1500mm까지 허용한다. 이미 같은 사람으로 묶인 그룹은 `person_group_rejoin_radius_mm=1500`, `person_group_attach_radius_mm=1600`으로 넓은 보폭을 유지한다

### 4.5 투기 감지 다중 경로 (Multi-Path Dumping Detection)

| 경로 | 메커니즘 | 게이트 |
| :--- | :--- | :--- |
| **Path 1** | 미매칭 클러스터 ↔ 궤적 이력 근접 | 600mm 이력, 200mm 현재 거리 |
| **Path 2** | 폭 감소 보조 신호 | 80mm threshold |
| **Path 3** | 클러스터 분열 보조 신호 | track width + 200mm margin |
| **Path 4** | 기존 정지 트랙 궤적 매칭 (Phase 6.5) | 궤적 + 보조 1개 |

### 4.6 오탐 방지 (False-Positive Guards)

1. **다리 근접 필터**: 350mm 이내 다른 사람 트랙 → suspect 취소 (source person 제외)
2. **크기 일관성**: suspect 기간 중 width 분산 > 2500mm² (σ > 50mm) → 취소
3. **Lost track recovery**: 600mm 이내 + 3 frame lost 이내 → 기존 물체 재등장으로 판단
4. **Deleted position buffer**: 삭제된 트랙 위치 30 frame 보존
5. **Hotspot 중복 차단**: 확정 투기 위치 500mm 반경, TTL 3000 frame (~5분)
6. **활성 투기물 30cm 차단**: 이미 확정된 투기물 근처 재감지 방지
7. **Birth-time gate**: 오래 있던 정지 물체는 지나가는 사람 궤적으로 suspect 전환 금지
8. **Source lock**: 생성 시점 source만 확정에 관여, passerby는 확정 근거에서 제외
9. **Pending cluster quarantine**: 미매칭 클러스터는 일반 3프레임, direct evidence 1프레임 관측 후 track/suspect 공개. 거의 움직이지 않는 정지 잔상은 1프레임 더 보류
10. **Visible suspect gate**: source lock + 신규 생성 + 직접 증거 + 4프레임 안정 전에는 카메라/Unity suspect callback 보류
11. **Fast confirm**: 신규 물체 + source lock + 직접 증거 + 300mm 이탈 + 3프레임 거리 증가 + 6프레임 안정이면 빠른 확정
12. **Strong depart distance**: 600mm 이상도 source lock/birth/direct evidence가 있을 때만 강한 확정 경로로 인정

### 4.7 현장 튜닝 절차

현장 튜닝은 한 번에 여러 값을 바꾸지 않는다. 아래 순서로 10Hz 로그를 2~3분씩 확인한다.

| 단계 | 증상 | 조정값 | 방향 |
|------|------|--------|------|
| 1 | 배경 edge/반사 spike가 계속 track으로 생김 | `static_bg_margin_mm`, `static_bg_min_confidence`, `static_bg_edge_guard_slots` | 기본 150/10/1 유지, 현장 심할 때만 margin 150→180, confidence 10→15, guard 1→2 |
| 2 | 1~2프레임 튐값이 suspect/capture를 만듦 | `pending_static_extra_frames`, `pending_cluster_confirm_frames`, `min_suspect_callback_frames` | static extra 1→2, pending 3→4, callback 4→5 |
| 3 | 배경 학습에서 빠진 정적 잔상이 오래 남음 | `ECOWARDEN_BG_RESIDUAL_FILTER_AFTER_FRAMES`, `ECOWARDEN_BG_RESIDUAL_FILTER_RADIUS_MM`, `ECOWARDEN_BG_RESIDUAL_FILTER_MAX_WIDTH_MM` | 기본 40프레임/140mm/350mm, 미탐이면 after 40→60 또는 90 |
| 4 | 실제 작은 병/봉투 감지가 늦거나 누락 | `fast_confirm_min_frames`, `person_depart_trend_frames`, `source_lost_confirm_frames`, `split_match_radius_mm` | fast 4 유지, trend 4 유지, source lost 8 유지, split match 250 유지 |
| 5 | 넓은 보폭에서 2명/2객체로 분리 | `person_group_wide_pair_radius_mm`, `person_group_wide_pair_max_depth_gap_mm`, `person_group_attach_radius_mm`, `person_group_rejoin_radius_mm`, `person_group_hold_frames` | 현재 wide radius 1500, Y gap 500, attach 1600, rejoin 1500, hold 14 |
| 6 | 가까운 두 사람이 한 명으로 병합 | `person_group_new_pair_max_width_mm`, `person_group_attach_radius_mm`, `person_group_rejoin_radius_mm`, `person_group_hold_frames` | new pair 650→600, attach 1200→1100, rejoin 1100→1000, hold 8→7 |
| 7 | source lost로 오탐 확정 | `source_lost_confirm_frames`, `kf_fallback_max_gate_mm` | source lost 8→12, KF cap 500→400 |

운영 기준:

- 오탐이 많으면 `precision` 값을 먼저 올린다: pending/callback/source_lost/static background 쪽을 강화한다.
- 실제 투기를 놓치면 `recall` 값을 제한적으로 낮춘다: 확정 프레임/이탈 프레임만 낮추고 source lock/direct evidence는 유지한다.
- `source_locked`, `newly_created_after_source`, `direct_dump_evidence` 세 조건은 현장 튜닝으로 끄지 않는다. 이 셋을 우회하면 passerby 오탐이 다시 열린다.

---

## 5. 데이터 형식 (Data Format)

### Unity 전송 (UDP JSON) — tracked_only 모드
매 프레임 **추적 확인된 객체만** 전송 (고스트/노이즈 제거):
```json
{
    "type": "FRAME",
    "frame_id": 42,
    "timestamp": 1712290800000,
    "objects": [
        {"id": 1, "x": 1200.0, "y": -350.0, "type": "normal", "width": 380.0, "is_abandoned": false},
        {"id": 2, "x": 500.0, "y": 800.0, "type": "dumped", "width": 120.0, "is_abandoned": true}
    ],
    "tracks": [{"id": 1, "x": 1200.5, "y": -350.2, "state": "moving",
         "is_dumped_item": false, "is_dump_suspect": false}],
    "points": [],
    "events": [{
        "type": "dumping",
        "person_track_id": 5, "person_x": 1200.0, "person_y": -800.0,
        "object_track_id": 6, "object_x": 1100.0, "object_y": -700.0,
        "timestamp": 1712290800000
    }]
}
```

- `objects.id` = 트랙 고유 ID (프레임 간 영속)
- `objects.type` = `"normal"` / `"object_candidate"` / `"dumped"` / `"suspect"` / `"abandoned"`
- `objects.width` = 클러스터 폭 (mm)
- `objects.is_abandoned` = Unity `LidarObject.is_abandoned` 호환 필드
- `points` = 빈 배열 (Unity는 트랙 위치만 사용)
- v10부터 일반 객체는 `confirmed=true`이고 `lost_count=0`인 트랙만 송신됨
- `suspect` / `dumped` 트랙은 투기 이벤트 흐름 보존을 위해 confirmed 게이트 예외로 송신됨

### 시각화기 전송 (UDP JSON) — 전체 모드
`127.0.0.1:9090`으로 전체 클러스터 + 점구름 전송 (디버그용). objects는 클러스터 인덱스 기반, points는 4:1 샘플링.

### Unity 바이너리 프로토콜 (UDP packed struct)
JSON과 동시 전송. **confirmed 트랙/group 기준 synthetic cluster만** 전송:
- `SendPoints()`: PacketHeader(20B) + PointItem(9B) x N (프래그먼트 분할)
- `SendClusters()`: PacketHeader(20B) + ClusterItem(25B) x N (트랙 메타데이터 포함)
- PersonGroup에 속한 leg cluster는 원본 물리 클러스터 2개를 보내지 않고 group 중심 synthetic cluster 1개로 보냄

### FastAPI 전송 (HTTP POST)
투기 확정 시 증거 데이터를 서버로 전송:
- **URL**: `https://api.ecowarden.systems/api/dumping-event`
- **Method**: POST
- **Headers**: `Content-Type: application/json`, `X-API-Key: <ECOWARDEN_API_KEY>` (환경변수로 주입 — 소스/문서에 실키를 쓰지 않는다)
```json
{
    "person_id": 5,
    "person_x": 1.2,
    "person_y": -0.8,
    "cumulative_dist": 3.5,
    "object_id": 6,
    "object_x": 1.1,
    "object_y": -0.7,
    "event_time": 1712290800000000000,
    "zone": 3,
    "severity": "high",
    "image_base64": "..."
}
```

- `zone` = 투기 발생 존(0~4), `severity` = 금지구역 내 투기면 `"high"`, 아니면 `"normal"` (v50 추가 필드 — 서버가 무시해도 무방)

금지구역 침입 시 (`intrusion-event`, §3.7):
```json
{
    "type": "intrusion",
    "person_id": 5,
    "person_x": 1.2,
    "person_y": -0.8,
    "zone": 3,
    "event_time": 1712290800000000000,
    "image_base64": "..."
}
```

---

## 6. 빌드 및 배포 (Build & Deploy)

```bash
# 빌드
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# 타겟
rplidar_app    # 프로덕션 바이너리
rplidar_sim    # 회귀 테스트 (exit code = fail count)
check_lidar    # USB 센서 테스트
check_camera   # 카메라 테스트
check_pir      # PIR 센서 테스트 (EKMC160111 x2, GPIO 17/27)
```

### 초기 설정 (Raspberry Pi)
```bash
# 전체 자동 설정 (의존성 + 권한 + 빌드)
chmod +x setup.sh && ./setup.sh
# 이후 반드시 로그아웃 → 재로그인 (GPIO/LiDAR 권한 적용)
# 즉시 PIR만 확인하려면 새 터미널에서 newgrp gpio 후 ./build/check_pir
```

`setup.sh`는 apt 패키지 설치, `/dev/ttyUSB*`/`/dev/gpiochip*` 권한 설정, Slamtec `rplidar_sdk` clone, CMake configure, 전체 타깃(`rplidar_app`, `rplidar_sim`, `check_lidar`, `check_camera`, `check_pir`, `servo_sweep_test`) 빌드와 산출물 확인까지 수행한다.

### 의존성
- **필수**: C++17, POSIX, libcurl, nlohmann/json (apt 우선, FetchContent 폴백)
- **선택**: OpenCV (camera), rplidar_sdk (vendored), libgpiod (PIR, RPi5 필수)
- **Python**: python3-pygame (시각화용, apt 설치)

### PIR 센서 GPIO 지원

| 플랫폼 | GPIO 백엔드 | 칩 이름 | 설치 |
|--------|------------|---------|------|
| RPi5 | libgpiod (자동 감지) | gpiochip4 (RP1) | `setup.sh` 또는 `apt install gpiod libgpiod-dev` |
| RPi4/3 | libgpiod 또는 sysfs fallback | gpiochip0 | 선택사항 |
| macOS (개발) | 스텁 (항상 motion=true) | - | 불필요 |

빌드 시 libgpiod가 없으면 자동으로 sysfs fallback 사용.

### 현장 런타임 튜닝

재컴파일 없이 환경변수로 일부 추적/확정 값을 조정할 수 있습니다. 기본은 v36 값입니다.

```bash
# 1) 먼저 로그만 켜서 어떤 gate에서 막히는지 확인
sudo env ECOWARDEN_LOG_LEVEL=dump ./build/rplidar_app

# 2) 보폭 split이 남을 때: 한 번에 하나만 증가
sudo env ECOWARDEN_PERSON_GROUP_HOLD_FRAMES=16 ./build/rplidar_app
sudo env ECOWARDEN_PERSON_GROUP_HOLD_ATTACH_RADIUS_MM=1900 ./build/rplidar_app
sudo env ECOWARDEN_PERSON_GROUP_WIDE_PAIR_RADIUS_MM=1600 ./build/rplidar_app

# 3) 쓰레기 suspect는 되지만 확정이 늦을 때
sudo env ECOWARDEN_FAST_CONFIRM_MIN_FRAMES=3 ./build/rplidar_app
sudo env ECOWARDEN_FAST_CONFIRM_DEPART_MM=250 ./build/rplidar_app
sudo env ECOWARDEN_SOURCE_LOST_CONFIRM_FRAMES=6 ./build/rplidar_app

# 4) 보조 신호가 약해서 suspect 자체가 안 생길 때
sudo env ECOWARDEN_TRAJECTORY_BIRTH_DEPART_MM=250 ./build/rplidar_app

# 5) 쓰레기와 사람이 한 객체로 붙으면 scan merge부터 낮춤
sudo env ECOWARDEN_SCAN_MERGE_RADIUS_MM=80 ./build/rplidar_app
```

오탐이 늘면 위 값을 역순으로 기본값에 가깝게 되돌립니다.

### systemd 배포
```ini
[Service]
Type=simple
ExecStart=/usr/local/bin/rplidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event 192.168.20.173:5005
Environment=ECOWARDEN_DRY_RUN=0
NoNewPrivileges=yes
ProtectSystem=strict
```

---

## 7. 테스트 (Testing)

### 회귀 스위트 (`test/sim_main.cpp`)

| # | 시나리오 | 프레임 | 내용 | 기대 |
|---|---------|--------|------|------|
| A | walk_by | 135 | 사람 1명 FOV 횡단 후 ID 삭제 | dump=0, final_tracks=0 |
| B | drop_and_leave | 130 | 봉투 투하 후 이탈 | dump=1, confirm [80,130] |
| C | two_people | 120 | 2인 교차 통과 | dump=0 |
| D | occlusion | 120 | 10 frame lost 후 재등장 | dump=0 |
| E | hotspot_traffic | 160 | 고빈도 통행 + Hotspot | dump=1 |
| F | background_change | 260 | 배경 변화 | dump=0 |
| G | frame_jitter | 120 | 프레임 지터 | dump=0 |
| H | multi_dump | 140 | 다중 투기 동시 발생 | dump=2 |
| I | large_split | 120 | 대형 클러스터 분열 | dump=0 |
| J | sensor_spike | 120 | 센서 노이즈 spike | dump=0 |
| K | reflective_spike_near_path | 120 | 사람 경로 근처 2~3프레임 반사 spike | dump=0 |
| L | background_edge_jitter | 160 | 배경 edge jitter 후 통행 | dump=0 |
| M | preexisting_object_passersby | 220 | 기존 정지 객체 앞 passerby | dump=0 |
| N | real_drop_with_noise | 130 | 실제 투기 + 주변 spike | dump=1 |
| O | quick_exit_drop | 130 | 투기 직후 source 빠른 이탈 | dump=1 |
| P | small_cup_drop | 150 | 소형 컵/소형 봉지급 투기 | dump=1 |
| Q | 20L_bag_drop | 150 | 20L급 봉투 투기 | dump=1 |
| R | 100L_bag_drop | 150 | 100L급 대형 봉투 투기 | dump=1 |
| S | large_preexisting_bag_passersby | 240 | 기존 대형 봉투 앞 통행 | dump=0 |
| T | stationary_person_not_trash | 150 | 정지/저속 사람 | dump=0 |
| U | close_delayed_split_drop | 150 | 사람 바로 옆 쓰레기 느린 분리 | dump=1 |
| V | very_quick_throw_exit | 130 | 투기 직후 2~3프레임만 보이고 이탈 | dump=1 |
| W | wide_stride_one_person | 135 | 다리 2개가 넓게 분리된 한 사람 | Unity normal=1, dump=0 |
| X | slow_stride_one_foot_stationary | 140 | 느린 보행/한쪽 발 정지 | Unity normal=1, dump=0 |
| Y | two_people_close_not_merged | 135 | 가까운 두 사람 오병합 방지 | Unity normal=2, dump=0 |
| Z | close_attached_drop_split | 150 | 사람 바로 옆 쓰레기가 scan merge로 붙지 않고 분리/확정 | dump=1 |
| AA | background_residual_passby | 235 | 배경 학습 직후 놓친 중간 크기 잔상 앞 통행 | Unity peak=1, final=0, dump=0 |
| BB | stationary_crowd_no_dump | 180 | 여러 사람이 같이 정지해도 사람 간격을 투기로 오해하지 않음 | dump=0 |
| ZP | zone_policy_unit | - | 금지구역 정책 단위 테스트 (파싱/연속 프레임/재알림/리셋/비활성) | 전체 assertion PASS |

**상태**: 29/29 PASS (v50 verified on Mac)

### 하드웨어 테스트

| 타겟 | 명령어 | 대상 |
|------|--------|------|
| `check_lidar` | `./build/check_lidar /dev/ttyUSB0` | RPLiDAR S2 USB 연결 |
| `check_camera` | `./build/check_camera` | USB 카메라 캡처 |
| `check_pir` | `./build/check_pir` | PIR 센서 GPIO 17/27 (libgpiod/sysfs) |
| `servo_sweep_test` | `sudo ./build/servo_sweep_test` | 서보 0~180도 5-zone sweep + 사진 저장 (Arduino/PWM 공통) |
| `servo_only_test` | `sudo ./build/servo_only_test` | Pi5 단독 PWM(GPIO18) 신호 진단 — 아두이노 없이 서보만 검증 |

---

## 8. 파일 구조 (File Map)

| 관심사 | 파일 |
|--------|------|
| **메인 루프** | `src/main.cpp` (LiDAR + Camera + PIR + 네트워크 통합) |
| **투기 감지 (추적)** | `src/cluster_tracker.cpp` (7-phase FSM) |
| **투기 감지 (판정)** | `include/dump_detector.h` (header-only) |
| **칼만 필터** | `include/kalman_filter.h` (header-only) |
| **LiDAR 인터페이스** | `src/rplidar_s2.cpp` |
| **클러스터링** | `src/scan_processor.cpp` (DBSCAN + grid) |
| **파라미터 단일 출처** | `include/production_params.h` |
| **이벤트 전송** | `src/event_notifier.cpp` (HTTP POST + retry) |
| **JSON 직렬화** | `src/json_packet.cpp` (UDP JSON 패킷) |
| **UDP 전송** | `src/udp_sender.cpp` (JSON + 바이너리 프로토콜) |
| **PIR 센서** | `src/pir_sensor.cpp` (libgpiod + sysfs fallback) |
| **카메라** | `src/camera_module.cpp` (PIMPL + async) |
| **서보 카메라 조준** | `src/servo_controller.cpp`, `include/servo_zone.h` |
| **성능 프로파일** | `include/phase_profiler.h` (CSV) |
| **회귀 테스트** | `test/sim_main.cpp` (28 scenarios) |
| **실시간 시각화** | `lidar_visualizer.py` (pygame, UDP 9090) |

---

## 9. 버전 이력 요약

마일스톤 단위로 묶었습니다. 세부 변경은 git log를 참조.

| 단계 | 버전 범위 | 핵심 주제 |
|------|-----------|----------|
| **증거 고도화 / 금지구역** | v49 ~ v50 | 서보 zone hysteresis/slew rate, LiDAR 재연결 루프, `SIGUSR1` 배경 리셋, API 키 env 주입, 금지구역(존) 침입 탐지(`ECOWARDEN_RESTRICTED_ZONES` + intrusion 이벤트), 블랙박스 전후 10초 AVI, 증거 사진 ID/존/시각 오버레이, dumping 페이로드 `zone`/`severity` |
| **서보 카메라 통합** | v39 ~ v48 | LiDAR 좌표 → 5-zone 서보 조준, Arduino USB serial backend, `setup.sh` PWM overlay 자동 설정, 사람별 촬영 세션(`captures/person_zones`) 보관/삭제, 카메라 live preview, suspect/confirm/dump bundle 저장 구조 |
| **현장 보폭/잔상 안정화** | v29 ~ v38 | 15cm 재설치 기준 보폭 split 완화(PersonGroup hold/attach/rejoin/wide pair 튜닝), 사람 옆 쓰레기 scan merge 분리·track hijack 방지, 배경 잔상 트랙 억제, suspect 쓰레기의 humanPrefab 송신 차단, PIR polling/권한 보강, dump gate debug 로그, Unity stale split 제거 |
| **탐지 회복 + 오탐 균형** | v21 ~ v28 | 30cm/15cm 설치 전제 재튜닝, 쓰레기 크기 band(Tiny~XL) 별 confirm gate, 100L 봉투 회귀 추가, source-lost 오탐 차단, 잔상/정지 사람 오탐 억제, 범위 이탈 사람 ID 수명 단축, Unity JSON 수신기 주소/형식 정합성 |
| **PersonGroup / 확정 경로** | v13 ~ v20 | PersonGroup/ObjectCandidate 도입(보폭으로 분리된 leg를 1명으로 송신), anti-phase `dot<=0` reject 제거, single-leg attach, source lock/birth gate/direct evidence, fast confirm, 배경·반사 spike 억제(static bg confidence + edge guard, pending cluster quarantine, visible suspect gate) |
| **고스트 트랙 / 데이터 흐름** | v7 ~ v12 | Unity tracked_only 모드, Tentative→Confirmed 승격 게이트, 배경 클러스터의 사람 객체 승격 차단, 정지 시 ID 유실/투기물 즉시 소멸 수정, 시각화기 UDP 분리, 학습 중 데이터 전송, 바이너리 프로토콜, PIR libgpiod 지원 |
| **정합성 / 인프라 기반** | v3 ~ v6 | KF Joseph form + adaptive gate, DumpDetector 분리, `production_params.h` 단일 출처, phase profiler, dry-run/time_utils/PIMPL/scratch 정리, 회귀 스위트 PASS |

### 트러블슈팅 이력

| 오류 | 원인 | 해결 |
|------|------|------|
| 시각화기에 데이터 안 보임 | 포트 불일치 (시각화기 9090, 앱은 5005만 전송) | 시각화기 전용 UDP 소켓 추가 (127.0.0.1:9090) |
| 배경 학습 카운트 0 | Process/Learn 동일 프레임 → 학습 즉시 필터 | Process 먼저, Learn은 다음 프레임용 |
| 학습 중 5초간 데이터 0 | bg_filter.Apply() false → continue로 전송 건너뜀 | continue 전에 시각화 데이터 전송 추가 |
| 오탐 (정지 객체 투기 판정) | EvaluateSuspects에서 정지=이탈로 처리 | 사람 트랙 실제 이탈 검증 필수 |
| 정지 시 ID 변경 | 적응형 배경이 정지 사람을 배경으로 흡수 + lost_age_limit 짧음 | 모든 활성 트랙 위치를 배경 필터에 전달하고, 복구는 `recovery_max_lost_frames` 안에서만 허용 |
| 투기물 즉시 사라짐 | FilterBackground slack 330mm + 150mm 무조건 삭제가 소형 투기물 제거 | slack 110mm로 축소, 무조건 삭제 제거, 추적 객체 보호 추가 |
| Unity에 배경 고스트 표시 | 사람 통과 시 배경 점이 1-2프레임 클러스터로 전송됨 | Unity에 tracked_only 모드 도입, 추적 객체만 전송 |
| 사람 1명이 여러 명으로 깜빡임 | 신규 트랙이 검증 없이 Moving으로 생성되고 Lost 상태 전환 없이 삭제 전까지 송신됨 | Tentative→Confirmed 게이트, lost 일반 트랙 미송신, JSON/바이너리 송신 기준 통일 |
| 배경/정적 물체가 사람 객체처럼 찍힘 | 현재 프레임의 모든 클러스터 슬롯을 배경 학습에서 차단해 배경으로 재흡수되지 못함 | 클러스터 전체 차단 제거, tracked-object 주변 슬롯만 학습 차단 |
| 보폭을 벌리면 여전히 2명처럼 보임 | JSON은 group 대표만 보내지만 바이너리 경로가 원본 다리 클러스터를 그대로 송신 | 바이너리도 group synthetic cluster 1개만 송신, PersonGroup hold 추가 |
| 봉투 투기 후 확정이 안 됨 | 600mm 이탈 필수, 5mm/frame 정지 게이트, 방향성 검사 때문에 봉투 흔들림/짧은 이탈에 막힘 | 450mm+거리 증가 추세 기본 확정, 600mm strong 확정, source lost 경로, 15mm/frame 정지 허용 |
| 보폭 anti-phase에서 여전히 2명으로 인식 | 양쪽 다리 순간 속도 dot product가 음수이면 group 생성을 거부 | dot hard reject 제거, group 반경 600mm로 확장, 기존 group hold 유지 |
| 기존 쓰레기 앞을 사람이 지나가면 투기로 확정 | 오래된 정지 객체도 궤적 매칭 + stationary_count로 suspect 전환 가능 | birth-time gate, source lock, 직접 증거 없는 오래된 정지 객체 suspect 금지 |
| 확정 속도가 느림 | 안정성/이탈을 2단계로 반복 확인 | fast confirm: 신규 물체 + source lock + 직접 증거 + 6프레임 안정으로 빠른 확정 |
| 배경/반사 때문에 중간중간 값이 튐 | 단일 static background EMA 슬롯과 foreground-only adaptive update가 edge/reflection을 안정적으로 구분하지 못함 | static background confidence/neighbor guard, adaptive two-pass, pending cluster quarantine, visible suspect gate |
| 넓은 보폭이 아직도 분리되고 확정도 늦음 | group 후보가 confirmed/moving leg에 치우치고 v16 precision gate가 direct evidence까지 지연 | leg-like attach 허용, attach/rejoin 1100/1000mm, hold 7프레임, direct evidence pending 1프레임 |
| 천천히 걸으면 사람 1명이 2개 객체로 보임 | 저속 다리가 `object_candidate`로 선분류되어 PersonGroup 후보에서 제외됨 | 느린 leg pair 후보 허용, group 확정 시 `object_candidate` 해제 |
| 중간중간 한 사람 옆에 작은 객체가 같이 보임 | group 없는 `object_candidate`가 Unity JSON `objects/tracks`에 별도 객체로 포함됨 | group 없는 `object_candidate`는 Unity JSON/바이너리 송신에서 제외 |
| 사람이 탐지 범위를 지나간 뒤 ID가 남음 | v8의 사람 track 수명 3배 확장이 범위 이탈 후에도 ID를 오래 보존 | 이동 사람 lost 수명을 `recovery_max_lost_frames=10`으로 제한, walk_by 종료 시 final_tracks=0 회귀 추가 |
| PIR Init Failed (RPi5) | RPi5 RP1 GPIO → sysfs 미지원 | libgpiod 1차 + sysfs fallback |
| `servo_sweep_test`에서 사진만 찍히고 서보가 안 돎 | Arduino sketch 미업로드, `/dev/ttyACM0` 불일치, 또는 공통 GND 누락 | Arduino에 `arduino/servo_zone_controller/servo_zone_controller.ino` 업로드, `ls /dev/ttyACM*`, `ECOWARDEN_SERVO_SERIAL_DEVICE` 확인 |
| `pip3 install pygame` 실패 | PEP 668 차단 | `sudo apt install -y python3-pygame` |

상세 분석은 `PROJECT_STATE.md` 참조.

---

## 10. 대전 지역 적용 시나리오 (지속가능발전 연계)

본 시스템은 CCTV 단독 감시의 한계(야간·사각지대 동선 파악 곤란, 상시 영상 촬영에 따른 사생활 침해)를 LiDAR 우선 감시로 보완하는 구조라, 대전 시내의 다음 지점에 그대로 적용할 수 있다.

| 적용 대상 | 활용 방식 |
|---|---|
| 국방 관련 시설·공공기관(한국철도공사 본사 등) 주변 보호구역 | 금지구역(존) 침입 탐지(§3.7)로 경계 접근·무단 투기를 동시 감시. 침입 시 `severity=high` 이벤트 + 전후 10초 영상 |
| 원도심 골목·상습 무단투기 지점 | 5-zone 서보 카메라가 LiDAR 감지 방향만 촬영 — 고정 CCTV 대비 사각지대 없이 저비용 상시 감시 |
| 갑천·유등천 등 하천변 투기 감시 | 야간 IR 카메라 + LiDAR 조합으로 조명 없는 구간에서도 투기 확정·증거 확보 |

대전 지속가능발전목표(D-SDGs, 대전지속가능발전협의회 tjla21.or.kr) 연계:

| D-SDGs 목표 | 연계 내용 |
|---|---|
| **목표 11 「안전하고 살기좋은 환경 조성」** (주) | 11-4 "도시환경에 미치는 부정적 요소를 감소" — 무단투기 실시간 적발·증거화로 도시환경 저해 요소 저감. 11-3 안전관리 역량 증대 — 보호구역 침입 상시 감시 |
| **목표 12 「자원의 효율적 생산·소비 추구」** (주) | 12-2 "폐기물 발생의 절대적 감소" / 12-3 재사용·재활용 활성화 — 상습 투기 억제로 올바른 분리배출 유도 |
| **목표 6 「건강하고 안전한 물관리」** (부) | 3대 하천(갑천·유등천·대전천) 관리·보호 — 하천변 투기 감시로 수질 오염원 차단 |
| **목표 16 「포용적 제도 구축」** (부) | 위·변조 불가(WORM) 증거 체계로 단속 행정의 투명성·신뢰성 보강 |

- **경제성**: 단속 인력 순찰 대비 저비용 상시 감시 — Raspberry Pi 5 기반 엣지 처리로 서버 부하 최소화
- **사회적 수용성**: 평상시에는 비식별 LiDAR 점군만으로 감시하고, 투기 의심/침입 시에만 카메라 증거를 남기는 **사생활 보호형 설계**. 투기 없이 사람이 지나가면 임시 사진은 자동 삭제된다(§3.6)
