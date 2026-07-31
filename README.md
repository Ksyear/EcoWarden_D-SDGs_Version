# EcoWarden: LiDAR 기반 사생활 보호형 불법 투기 감지 시스템 (Embedded)

불법 쓰레기 투기가 잦은 **사각지대**(원도심 골목, 하천변, 상가 후면, 이면도로 등 사람 통행이 적은 지점)를 대상으로 하는 불법 투기 감지 시스템의 임베디드 모듈입니다. 평상시에는 카메라를 녹화하지 않고 RPLiDAR S2가 **비식별 좌표만으로** 이동 객체를 추적하다가, 투기 행위가 확정되는 순간에만 서보 카메라를 해당 방향으로 조준해 증거(오버레이 사진 + 전후 10초 블랙박스 영상)를 남기고 서버로 전송합니다. 24시간 영상을 상시 저장하는 CCTV 대비 저장공간·사생활 침해·사후 확인 부담을 줄이는 구조입니다.

핵심 기능:

1. **비식별 통행 추적 + 사람↔물체 구별** — 10Hz LiDAR 점군 → DBSCAN 클러스터링 + 칼만 필터 추적 → Unity 대시보드에 트랙 ID·이동 경로 실시간 표시. 트래커가 사람(PersonGroup)과 투기물(object)을 분리
2. **불법 투기 감지** — 다단계 투기 판정 FSM으로 투기 행위 확정·증거화 (§4.6). LiDAR/카메라는 **1차 스크리닝**으로 후보를 잡고, 최종 판단은 관리자가 증거 사진으로 확인하는 구조
3. **증거 보존** — ID/UTC시각 오버레이 사진 + 사람 머리 위 `ID:` 라벨, 이벤트 전후 10초 블랙박스 AVI, FastAPI 전송 + 실패 시 로컬 큐잉 (§3.8)
4. **프라이버시 3계층 (v56)** — ① 평상시 무촬영 ② 증거 사진 **얼굴 자동 마스킹**(fail-closed) ③ 마스킹 전 원본 **AES-256-GCM 암호화 보관 + 보존기한 자동 파기 + 열람 로그** (§3.9)
5. **객체 분류 · 이벤트 등급 (v56)** — 사람/동물/불명 분류 후 `high`/`medium`/`low`/`ignore` 등급 산정. 카메라 DNN 백엔드 + LiDAR 기하 휴리스틱 2단 구조 (§3.10)
6. **확장(옵션): 금지구역 침입 감지** — 5-zone 금지구역 정책, 침입 확정 시 `intrusion` 이벤트 + 서보 조준 촬영 (§3.7). **기본 비활성**(`ECOWARDEN_RESTRICTED_ZONES` 설정 시 활성) — 하천변·시설 보호구역 확장용

탐지는 **지능형 다중 센서 융합 알고리즘**(DBSCAN 클러스터링 + 칼만 필터 추적 + 존 정책 + 다단계 판정 FSM) 기반입니다. v56에서 사람/동물/불명 **객체 분류기**가 추가됐는데, 기본 동작은 여전히 규칙 기반(LiDAR 기하 휴리스틱)이며 **학습형 신경망은 모델 파일을 명시적으로 지정했을 때만** 동작합니다(`ECOWARDEN_CLASSIFY_MODEL`). 모델이 없으면 휴리스틱으로 자동 폴백하고 신뢰도 상한이 낮게 적용됩니다 — 즉 "AI 기반"이라는 표현은 모델을 붙였을 때만 정확합니다.

**현재 버전**: v56 — 불법 투기 감지(주) + **프라이버시 3계층(얼굴 마스킹 · 원본 암호화 · 자동 파기)** + **객체 분류/이벤트 등급** + 블랙박스 전후 10초 영상 + 증거 사진 ID/시각 오버레이 + 머리 위 ID 라벨 + 침입 감지(옵션) (자세한 변경 이력은 §9 참조)
**포지셔닝**: **불법 투기 감지 시스템**(주). 2026-07-04 문서상 '보안 감시'로 전환했다가 **2026-07-08 불법 투기로 재확정** — 금지구역 침입 감지는 같은 플랫폼의 **옵션 확장 기능**(기본 OFF)으로 보존. 발표·시나리오·통신 명세는 `보안/` 폴더 참조.

---

## 1. 하드웨어 사양 (Hardware Info)

| 구분 | 상세 모델 및 규격 | 용도 |
| :------------ | :----------------------------------------------------------------- | :------------------------------------ |
| **LiDAR** | Slamtec RPLiDAR S2 (ToF, 30m) | 실시간 거리 측정, 객체 탐지 |
| **Camera** | [DFROBOT] 2MP USB Night Vision [FIT0730] | 비동기 30fps 증거 캡처 (1080p, IR 지원) |
| **Servo** | SG90급 3선 hobby servo (기본: Arduino USB serial / Pi5 단독: GPIO18 PWM) | suspect 좌표 기준 카메라 5-zone pan 조준 |
| **Sensor** | Panasonic PaPIRs (EKMC160111) x 2 | 정밀 디지털 모션 감지 (좌: GPIO 17, 우: GPIO 27) |
| **Computing** | Raspberry Pi 5 + Ubuntu Linux | 핵심 로직 및 통신 처리 |

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
│ ├─ FilterNoise (distance/intensity/FOV gate)
│ ├─ FilterStaticBackground (confidence + neighbor edge guard)
│ ├─ ToCartesian (polar → x/y mm)
│ ├─ DBSCAN (ε=100mm, min_pts=3, grid O(n))
│ └─ LearnStaticBackground (EMA alpha=0.02, tracked-object 슬롯 + edge guard 보호)
│
├─ BackgroundFilter
│ ├─ LearnFrame (50-frame warmup, 30% threshold)
│ ├─ FilterBackground (radius: 80mm)
│ └─ 학습 중에도 시각화 데이터 전송 (점구름 + 빈 트랙)
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
│ └─ 사람 트랙의 존이 금지 존에 연속 N프레임 관측되면 intrusion 후보
         │
├─ ObjectClassifier (v56, §3.10)
│ ├─ DNN 백엔드 (ECOWARDEN_CLASSIFY_MODEL 지정 시) ─┐
│ ├─ LiDAR 기하 휴리스틱 (폴백, 신뢰도 상한 0.7) ───┤
│ └─ GradeEvent() → high / medium / low / ignore ◄─┘
│ └─ ignore(noise) 면 이벤트를 전송하지 않고 종료
         │
Output Layer
├─ JsonPacketSender → Unity UDP (JSON, 기본 192.168.20.102:5005)
├─ JsonPacketSender → 시각화기 UDP (JSON, 127.0.0.1:9090)
├─ UdpSender → Unity 바이너리 프로토콜 (옵션, ECOWARDEN_UNITY_BINARY=1)
├─ EventNotifier → HTTP POST dumping-event / intrusion-event (X-API-Key)
│ └─ v56: 키 없으면 전송 안 하고 로컬 큐 보존 (fail-closed)
├─ ServoController → suspect/person 좌표를 5-zone으로 변환해 카메라 조준/저장
└─ CameraModule
   ├─ 사진 경로  PrepareEvidenceFrame() → JPEG
   │    (1) EvidenceVault : 마스킹 前 원본을 AES-256-GCM 암호화 보관 (§3.9)
   │    (2) FaceMasker    : 얼굴 검출 → 블러 + 번호(F1,F2…) 새김, fail-closed
   │    (3) 배너·머리 위 ID 라벨 렌더링   ← 반드시 마스킹 後
   │    (4) <이미지>.masks.json 사이드카에 번호·좌표 기록
   └─ 영상 경로  BlackboxSaveWorker() → MJPG AVI
        (1) 링버퍼는 원본 JPEG 보관 (메모리에만, 파일로 남지 않음)
        (2) 디스크에 쓰기 직전 프레임마다 FaceMasker 적용
        →  저장·업로드되는 사진과 영상은 항상 "마스킹본"
```

> **순서가 중요하다**: 마스킹을 라벨보다 나중에 하면 ID 라벨까지 블러 처리된다.
> 그리고 마스킹이 요구됐는데 검출기가 없으면 `PrepareEvidenceFrame()` 이 false 를
> 돌려 **사진 저장 자체를 건너뛴다** — 원본이 새어 나갈 경로를 만들지 않는다.
>
> **영상도 마찬가지다.** 블랙박스 링버퍼는 10Hz 상시 인코딩이라 거기서 얼굴 검출까지
> 돌리면 캡처 스레드에 부담이 크다. 그래서 **파일로 쓰는 순간**에만 마스킹한다 —
> 원본 프레임은 메모리에만 존재하고 디스크에는 남지 않는다.

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
| 3 | Unity 주소 | `192.168.20.102:5005` | 와이파이 변경 시 IP 수정 필요 |
| 4 | 시각화기 주소 | `127.0.0.1:9090` | localhost이므로 변경 불필요 |

### 3.3 실행 예시

```bash
# 기본값으로 실행 (Unity IP가 192.168.20.102일 때)
./build/rplidar_app

# Unity PC IP가 바뀌었을 때
./build/rplidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event 192.168.20.102:5005

# 시각화기 (별도 터미널) — 레이더 화면 + 우측 '수신 데이터 값' 패널
# d: 데이터 패널 토글, c: 이벤트 로그 클리어, q/ESC: 종료
# 침입(intrusion) 이벤트 수신 시 해당 존을 빨간 부채꼴로 5초 강조
python3 lidar_visualizer.py
```

### 3.4 네트워크 구조

```
rplidar_app ──UDP JSON──→ Unity PC (192.168.20.102:5005) ← 와이파이 IP 의존
            ──UDP 바이너리→ Unity PC (ECOWARDEN_UNITY_BINARY=1일 때만)
            ──UDP JSON──→ localhost:9090 (시각화기) ← 와이파이 무관
            ──HTTPS────→ api.ecowarden.systems (FastAPI) ← DNS, 와이파이 무관
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
| `ECOWARDEN_DUMP_VALIDATE` | 기본 `1` | 투기 확정 후 재검증 (§4.6.1). `0`이면 기존처럼 확정 즉시 전송 |
| `ECOWARDEN_DUMP_VALIDATE_FRAMES` | 기본 `30` | 확정 후 투기물 잔존 관찰 프레임 수 (10Hz 기준 약 3초) |
| `ECOWARDEN_DUMP_VALIDATE_MAX_MOVE_MM` | 기본 `300` | 관찰 중 투기물이 이 이상 이동하면 취소 (정지 투기물 아님) |
| `ECOWARDEN_DUMP_VALIDATE_MIN_PRESENT` | 기본 `0.5` | 관찰 창 최소 재관측 비율 — 미달 시 고스트/반사로 보고 취소 |
| `ECOWARDEN_DUMP_VALIDATE_HIGH_PRESENT` | 기본 `0.8` | `confidence="high"` 재관측 비율 기준 (투기자 이탈 동반 시) |
| `ECOWARDEN_DUMP_VALIDATE_DEPART_MM` | 기본 `1200` | 투기 주체가 투기물에서 이 이상 멀어지면 "이탈" 판정 |
| `ECOWARDEN_HEAD_LABEL_ENABLE` | 기본 `1` | 증거 사진의 사람 머리 위 ID 라벨 (§3.6) |
| `ECOWARDEN_CAMERA_HFOV_DEG` / `ECOWARDEN_CAMERA_VFOV_DEG` | `62` / `38` | 카메라 수평/수직 화각 — 라벨 위치 보정용 |
| `ECOWARDEN_CAMERA_PITCH_DEG` | 기본 `15` | 카메라 상향 기울기. 15cm 높이 설치에서 얼굴을 찍으려면 위로 기울여야 하며 실측각을 입력 |
| `ECOWARDEN_CAMERA_HEIGHT_M` / `ECOWARDEN_HEAD_HEIGHT_M` | `0.15` / `1.60` | 카메라 설치 높이 / 가정 머리 높이 |
| `ECOWARDEN_HEAD_LABEL_IMG_MIRROR` | 기본 `0` | 사진 좌우가 실물과 반대면 `1` |
| `ECOWARDEN_BLACKBOX_ENABLE` | 기본 `1` | 이벤트 전후 영상 링버퍼 활성 (§3.8) |
| `ECOWARDEN_BLACKBOX_SEC` | 기본 `10` | 이벤트 전/후 저장 구간(초). `_PRE_SEC`/`_POST_SEC`로 개별 지정 가능 |
| `ECOWARDEN_BLACKBOX_FPS` | 기본 `10` | 링버퍼 프레임 레이트. 메모리와 트레이드오프 |
| `ECOWARDEN_CLIP_UPLOAD` | 기본 `1` | 블랙박스 클립 서버 업로드 (§3.8). 서버 `evidence-clip` 준비 전 `0` |
| `ECOWARDEN_CLIP_URL` | (자동 유도) | 클립 업로드 endpoint. 미설정 시 dumping URL에서 `evidence-clip`으로 치환 |
| `ECOWARDEN_CLIP_TIMEOUT_MS` | 기본 `120000` | 클립(수십 MB) 전용 업로드 타임아웃 |
| `ECOWARDEN_API_KEY` | (비밀값) | FastAPI `X-API-Key`. 미설정 시 노출된 레거시 키로 fallback + 경고 로그 |
| `ECOWARDEN_QUEUE_FILE` | 기본 `/tmp/rplidar_event_queue.jsonl` | 전송 실패 이벤트 큐 파일 경로. 운영 배포는 `/var/lib/ecowarden/` 권장 |

#### 프라이버시 2계층 — 얼굴 마스킹 (v56, §3.9)

| 환경변수 | 기본값 | 설명 |
|------|------|------|
| `ECOWARDEN_FACE_MASK` | `1` | 얼굴 마스킹 on/off. **끄면 원본이 그대로 전송된다** |
| `ECOWARDEN_FACE_MASK_MODE` | `blur` | `blur` \| `pixelate` \| `box` |
| `ECOWARDEN_FACE_MASK_REQUIRE` | `1` | **fail-closed** — 검출기 없으면 원본을 내보내지 않음 |
| `ECOWARDEN_FACE_MASK_FALLBACK` | `1` | 검출기 없을 때 상단 영역 통째 마스킹. `0`이면 사진 저장 자체를 건너뜀 |
| `ECOWARDEN_FACE_MASK_FALLBACK_RATIO` | `0.45` | 폴백 마스킹 영역 (화면 높이 대비) |
| `ECOWARDEN_FACE_MASK_CASCADE` | (자동 탐색) | Haar cascade XML 경로 |
| `ECOWARDEN_FACE_MASK_DNN_CONFIG` / `_DNN_WEIGHTS` | (없음) | DNN 백엔드. 둘 다 지정 시 Haar보다 우선 |
| `ECOWARDEN_FACE_MASK_STRENGTH` | `0.35` | blur 강도 (ROI 짧은 변 대비 커널 비율) |
| `ECOWARDEN_FACE_MASK_BLOCKS` | `10` | pixelate 블록 수 (작을수록 강함) |
| `ECOWARDEN_FACE_MASK_EXPAND` | `0.25` | 검출 박스 확장 비율 (머리카락·턱선 포함) |
| `ECOWARDEN_FACE_MASK_MIN_PX` | `24` | 이보다 작은 얼굴은 무시 |
| `ECOWARDEN_FACE_MASK_DETECT_WIDTH` | `640` | 검출용 축소 폭(px). 0이면 원본 해상도에서 검출(느림) |
| `ECOWARDEN_FACE_MASK_LABEL` | `1` | 마스킹 자리에 번호(F1, F2…) 새김 |
| `ECOWARDEN_FACE_MASK_LABEL_PREFIX` | `F` | 번호 접두사 |
| `ECOWARDEN_FACE_MASK_LABEL_SCALE` | `0.8` | 번호 글자 크기 |

#### 프라이버시 3계층 — 원본 암호화 보관 (v56, §3.9)

| 환경변수 | 기본값 | 설명 |
|------|------|------|
| `ECOWARDEN_EVIDENCE_VAULT` | `0` | 원본 보관 on/off. **켜려면 키가 필요** (없으면 fail-closed) |
| `ECOWARDEN_EVIDENCE_KEY` | (비밀값) | AES-256 키 hex 64자. `openssl rand -hex 32` |
| `ECOWARDEN_EVIDENCE_KEY_FILE` | `/etc/ecowarden/secrets.conf` | 키 파일 경로 |
| `ECOWARDEN_EVIDENCE_DIR` | `captures/vault` | 암호화 원본 저장 위치 |
| `ECOWARDEN_EVIDENCE_LOG` | `captures/vault/access.log` | 열람 로그 (append-only) |
| `ECOWARDEN_EVIDENCE_RETAIN_DAYS` | `30` | 보존 일수. `0`이면 무기한(비권장) |

#### 객체 분류 · 이벤트 등급 (v56, §3.10)

| 환경변수 | 기본값 | 설명 |
|------|------|------|
| `ECOWARDEN_CLASSIFY` | `1` | 분류 on/off |
| `ECOWARDEN_CLASSIFY_MODEL` | (없음) | DNN 모델(.onnx/.caffemodel). **지정해야 "AI 기반"이 정확** |
| `ECOWARDEN_CLASSIFY_CONFIG` / `_LABELS` | (없음) | DNN 설정·라벨 파일 |
| `ECOWARDEN_CLASSIFY_CONF` | `0.5` | DNN 신뢰도 임계 |
| `ECOWARDEN_CLASSIFY_LIDAR_CAP` | `0.7` | LiDAR 휴리스틱 신뢰도 **상한** (과장 방지) |
| `ECOWARDEN_CLASSIFY_PERSON_MIN_W` / `_MAX_W` | `150` / `900` | 사람 폭 대역 (mm) |
| `ECOWARDEN_CLASSIFY_PERSON_MIN_V` / `_MAX_V` | `300` / `2200` | 사람 보행 속도 대역 (mm/s) |
| `ECOWARDEN_CLASSIFY_ANIMAL_MAX_W` | `450` | 동물 폭 상한 (mm) |
| `ECOWARDEN_CLASSIFY_ANIMAL_MIN_V` | `400` | 동물 최소 속도 (mm/s) |
| `ECOWARDEN_CLASSIFY_NOISE_AGE` | `3` | 이보다 짧게 관측되면 noise |
| `ECOWARDEN_CLASSIFY_PERSON_FRAMES` | `5` | 사람 확정 최소 매칭 프레임 |
| `ECOWARDEN_CLASSIFY_STATIC_FRAMES` | `10` | 정지 물체 판정 프레임 |

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

**사람 머리 위 ID 라벨 (v53)**: 사람 존 촬영 사진에는 배너와 별도로, 사람 머리 위 추정 위치에 `ID:<트랙ID>` 라벨(초록 박스 + 포인터 라인)이 그려진다. 위치는 LiDAR 좌표로 계산한다 — 수평은 서보 조준각 대비 사람 각도 차, 수직은 거리와 (머리높이−카메라높이)의 고도각에서 카메라 피치를 뺀 값 (`include/head_label.h`). 여러 사람이 프레임에 있어도 어떤 사람이 해당 트랙 ID인지 사진만으로 특정할 수 있다. 라벨이 어긋나면 `ECOWARDEN_CAMERA_PITCH_DEG`(실측 기울기)부터 보정하고, 좌우가 반대면 `ECOWARDEN_HEAD_LABEL_IMG_MIRROR=1`.

### 3.7 금지구역(존) 침입 탐지 — 확장(옵션) 기능

같은 플랫폼의 **옵션 확장 기능**이다(기본 비활성). 보호구역·출입 제한 구역을 존 단위로 지정하면, LiDAR가 추적하는 사람이 해당 존에 들어왔을 때 투기 여부와 무관하게 침입 이벤트를 만든다. 하천변 관리시설·설비 보호구역 등으로의 확장 시나리오에 사용한다.

```bash
# 존 3, 4를 금지구역으로 지정 (존 번호는 §3.6의 5-zone과 동일: 0~4)
sudo env ECOWARDEN_RESTRICTED_ZONES=3,4 ./build/rplidar_app
```

침입 확정 시 동작:

1. 서보 카메라를 침입자 존으로 즉시 조준·촬영
2. FastAPI `intrusion-event`로 `person_id`, `zone`, 좌표, 시각, 사진(base64) 전송
3. Unity/시각화기 FRAME `events[]`에 `intrusion` 이벤트 송신 (`severity:"high"`, UDP 손실 대비 5프레임 반복 — v51)
4. `captures/intrusions/`에 블랙박스 영상(전후 10초) 저장 → 저장 완료 시 서버로 자동 업로드 (`evidence-clip`, v54)
5. 해당 존에서 투기가 확정되면 dumping 이벤트의 `severity`가 `"high"`로 표기

오탐 방지: 존 경계 각도 노이즈로 1~2프레임 금지 존에 걸치는 경우를 걸러내기 위해 기본 3프레임 연속 관측을 요구하고(`ECOWARDEN_INTRUSION_MIN_FRAMES`), 같은 사람은 기본 10초 간격으로만 재알림한다(`ECOWARDEN_INTRUSION_REPEAT_MS`). 서버가 intrusion endpoint를 아직 지원하지 않으면 `ECOWARDEN_INTRUSION_NOTIFY=0`으로 전송만 끌 수 있다 (촬영·영상·로그는 유지).

### 3.8 블랙박스 증거 영상 (전후 10초)

카메라 캡처 스레드가 최근 프레임을 JPEG로 압축해 메모리 링버퍼에 상시 보관하다가, 투기 확정 또는 금지구역 침입 시 **이벤트 전 10초 + 후 10초** 구간을 AVI(MJPG)로 저장한다. 정지 사진만으로는 투기 순간의 행위 연속성을 입증하기 어렵다는 한계를 보완한다.

- 투기 확정: `captures/dumps/<event>/blackbox.avi` (suspect/confirm 사진, meta.json과 같은 번들)
- 금지구역 침입: `captures/intrusions/intrusion_<person>_<ts>.avi`
- 저장은 별도 worker thread에서 수행되어 10Hz scan loop를 막지 않는다
- 기본 10초/10fps/JPEG 75 기준 링버퍼 메모리는 1080p에서 약 40~60MB (Pi5 8GB 여유 범위)
- `ECOWARDEN_BLACKBOX_SEC`(전/후 공통), `_PRE_SEC`/`_POST_SEC`(개별), `_FPS`, `_ENABLE`로 조정

**서버 업로드 (v54)**: 클립 저장이 끝나면 전용 업로드 스레드가 `multipart/form-data`로 FastAPI `evidence-clip` endpoint에 자동 전송한다 (`event_type`/`person_id`/`event_time` 필드로 이벤트 레코드와 연결). 사진(base64)은 이벤트 순간 즉시, 영상은 완성 후(+10초~) 따라가는 2단 구조다. 업로드가 실패해도 파일은 기기에 남고, 서버 준비 전에는 `ECOWARDEN_CLIP_UPLOAD=0`으로 끈다. 형식 상세는 [`보안/통신_명세.md`](보안/통신_명세.md) §4.4.

### 3.9 프라이버시 3계층 — 얼굴 마스킹 · 원본 암호화 (v56)

증거 이미지가 기기 밖으로 나가거나 디스크에 남는 **모든 경로**에 동일하게 적용된다 — 사진뿐 아니라 **블랙박스 영상도 포함**이다.

| 계층 | 내용 | 구현 | 기본값 |
|------|------|------|--------|
| **1계층 평상시** | 카메라 미작동, 비식별 LiDAR 좌표만 | 구조적 보장 | 항상 |
| **2계층 공개본** | 증거 사진·영상 **얼굴 자동 마스킹 + 번호 새김** 후 저장·전송 | `include/face_masking.h` | **ON** |
| **3계층 원본** | 마스킹 전 원본 **AES-256-GCM 암호화** + 보존기한 자동 파기 + 열람 로그 | `include/evidence_vault.h` | OFF (키 설정 후 켬) |

**적용 범위**

| 경로 | 함수 | 마스킹 |
|------|------|--------|
| 서버 전송용 base64 | `CaptureBase64()` | 적용 |
| 일반 스냅샷 | `CaptureToFile()` | 적용 |
| 증거 사진 (오버레이) | `CaptureToFilePath()` | 적용 + 사이드카 JSON |
| **블랙박스 영상** | `BlackboxSaveWorker()` | **적용** (프레임마다) |
| 로컬 디버그 프리뷰 | `ShowPreviewFrame()` | 미적용 — 화면 표시 전용, 저장·전송하지 않음 |

**처리 순서** — `CameraModule::PrepareEvidenceFrame()`:
1. 원본 JPEG를 vault에 암호화 저장 (마스킹 전 상태여야 3계층의 의미가 있음)
2. 얼굴 검출 → 블러/모자이크/박스 처리
3. 그 다음에 배너·머리 위 ID 라벨을 그린다 (반대로 하면 라벨까지 블러됨)

블랙박스 영상은 `BlackboxSaveWorker()` 가 **파일로 쓰기 직전** 프레임마다 마스킹한다. 링버퍼는 10Hz 상시 인코딩이라 거기서 검출까지 돌리면 캡처 스레드 부담이 크기 때문이다. 원본 프레임은 메모리에만 존재하고 디스크에 남지 않는다. 검출기는 스레드 안전하지 않아 사진 경로와 **별도 인스턴스**를 쓴다.

#### 마스킹 번호 (F1, F2 …)

얼굴을 가리면 사진만으로 누가 누구인지 구별할 수 없다. 관리자가 "1번이 버렸다"고 지목할 수 있어야 증거로 의미가 있으므로, **마스킹한 자리에 번호를 새긴다.**

- 번호는 화면 **왼쪽부터** 매긴다 — 같은 장면이면 프레임이 바뀌어도 번호가 대체로 유지된다
- 마스킹 영역에 테두리를 그려 **어디를 가렸는지** 보이게 한다
- `ECOWARDEN_FACE_MASK_LABEL=0` 으로 끄거나 `_LABEL_PREFIX` 로 접두사를 바꿀 수 있다

#### 사이드카 메타 — `<이미지경로>.masks.json`

번호가 화면 어디에 해당하는지 서버·대시보드가 기계적으로 다루려면 좌표가 필요하다. 증거 사진 자체는 건드리지 않고 같은 이름의 JSON을 나란히 남긴다.

```json
{
  "image": "20260731_153000_zone_3.jpg",
  "backend": "haar",
  "used_fallback": false,
  "face_count": 2,
  "label_prefix": "F",
  "faces": [
    {"label": "F1", "index": 1, "x": 412, "y": 180, "w": 160, "h": 190},
    {"label": "F2", "index": 2, "x": 980, "y": 205, "w": 145, "h": 172}
  ]
}
```

#### 검출 성능

1080p를 그대로 Haar에 넣으면 프레임당 수십~수백 ms라 블랙박스 200프레임 처리가 현실적이지 않다. 그래서 **축소본에서 검출하고 박스를 원본 좌표로 되돌린다** — 마스킹 자체는 항상 원본 해상도에 적용된다. 기본 검출 폭 640px, `ECOWARDEN_FACE_MASK_DETECT_WIDTH` 로 조정한다.

**fail-closed 설계**: 마스킹이 켜져 있는데 검출기를 로드하지 못하면 원본을 그대로 내보내지 않는다. `ECOWARDEN_FACE_MASK_FALLBACK=1`(기본)이면 화면 상단을 통째로 마스킹하고, `0`이면 **사진 저장 자체를 건너뛴다**. "마스킹한다고 말해 놓고 실제로는 원본이 나가는" 상황을 구조적으로 막는다.

검출기 우선순위: DNN(`_DNN_CONFIG`+`_DNN_WEIGHTS` 지정 시) → Haar cascade(자동 탐색) → fail-closed 폴백.

**vault 파일 포맷**: `EWV1` 매직 + 생성시각 + 보존일수 + nonce + GCM tag + 암호문. 헤더를 AAD로 묶어 **메타데이터 위·변조도 검출**된다(목표 16 증거 무결성 근거). 복호화할 때마다 `access.log`에 시각·사유·요청자가 append된다.

```bash
# 키 생성 후 3계층 활성화
openssl rand -hex 32 # 64자 hex 출력
echo "ECOWARDEN_EVIDENCE_KEY=<위 값>" | sudo tee -a /etc/ecowarden/secrets.conf
export ECOWARDEN_EVIDENCE_VAULT=1
```

OpenSSL 없이 빌드하면 vault는 **평문 폴백 없이 비활성**된다 (원본을 아예 저장하지 않음).

### 3.10 객체 분류 · 이벤트 등급 (v56)

`보안/전체_구조.md` §5·§6이 스펙만 정의하고 비어 있던 부분의 구현이다.

**2단 백엔드**:
| 백엔드 | 조건 | 신뢰도 |
|--------|------|--------|
| `dnn` | `ECOWARDEN_CLASSIFY_MODEL` 지정 + OpenCV dnn 모듈 | 모델 출력 그대로 |
| `lidar_geom` | 항상 사용 가능 (폴백) | `ECOWARDEN_CLASSIFY_LIDAR_CAP`(기본 0.7) 상한 |

**등급 산정** (`GradeEvent`):
| 분류 | severity | 이벤트 타입 |
|------|----------|------------|
| `person` (conf ≥ 0.5) | `high` | `intrusion` |
| `person` (conf < 0.5) | `medium` | `intrusion` |
| `unknown` | **`medium`** | `unknown_object` |
| `animal` / `static_object` | `low` | `animal_or_small_object` |
| `noise` | `ignore` (전송 안 함) | — |

금지구역 안이면 한 단계 상향(`low`→`medium`, `medium`→`high`). `noise`는 상향하지 않는다.

**설계 원칙 — "모르면 올린다"**: `unknown`은 `low`가 아니라 `medium`이다. 미탐(진짜 사건을 놓침)이 오탐보다 비싸다는 프로젝트 전제와 같다.

**사람↔동물 겹침 대역 처리**: 15cm 높이에서 중형견 단면(~250mm)과 사람 다리 하나(~150mm)는 2D LiDAR로 구분되지 않는다. 그래서 겹치는 구간은 **동물로 내리지 않는다** — 동물로 잘못 내리면 severity가 `low`로 떨어져 진짜 사람을 놓치기 때문이다. 동물 판정은 ① 사람 최소폭 미만 + 이동 중, ② 보행 속도 상한 초과 + 좁은 단면 — 두 경우만 한다. 회귀 테스트 `OC_object_classifier_unit` Case D3이 이 동작을 고정한다.

### 3.11 증거 사진 HTTP 서버 (`pi_photo_server.py`)

백엔드(FastAPI)가 준비되지 않았을 때 **Unity 가 기기에서 사진을 직접 받아가는 우회 경로**다. 임베디드 C++ 은 관여하지 않는다 — `rplidar_app` 이 `captures/` 에 저장한 JPEG 를 읽어 서빙만 한다. 표준 라이브러리만 쓴다.

```bash
# rplidar_app 과 같은 작업 디렉터리에서
python3 pi_photo_server.py                      # 포트 8088, ./captures
python3 pi_photo_server.py --root /opt/ecowarden/captures --token <값>

# 상시 운영
sudo cp deploy/ecowarden-photo.service /etc/systemd/system/
sudo systemctl enable --now ecowarden-photo
```

| 엔드포인트 | 용도 |
|---|---|
| `GET /file/<상대경로>` | `dumping` 이벤트의 `image_file` 로 지목된 **정확한 사진** |
| `GET /latest.jpg` | 가장 최근 이미지 (`image_file` 이 없을 때 폴백) |
| `GET /health` | 상태 확인 (파일 개수, 최신 파일, 인증 여부) |
| `GET /list` | 최근 20개 목록 |

**접근 제한 — 반드시 알아둘 것**

`captures/vault/` 에는 마스킹 **전** 원본의 암호문(`.ewv`)과 **열람 로그**(`access.log`)가 쌓인다. 그래서 이 서버는 두 겹으로 막는다.

1. **`vault/` 디렉터리 전면 차단** — 이미지 확장자여도 403
2. **이미지 확장자만 서빙** — `.masks.json`, `.avi`, `meta.json` 등은 403

경로 탈출(`../`)도 `commonpath` 로 차단한다.

`--token` (또는 `ECOWARDEN_PHOTO_TOKEN`)을 주면 `X-Auth-Token` 헤더나 `?token=` 을 요구한다. **설정하지 않으면 LAN 에 붙은 누구나 증거 사진을 가져갈 수 있다** — 시연망이라도 켜 두는 편이 낫다.

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

### 4.5 금지구역 침입 판정 (Zone Policy) — 확장(옵션) 기능

- **존 매핑**: 사람 트랙(그룹이면 그룹 대표) 좌표를 서보 조준과 동일한 기준(`SuspectToZone`, atan2)으로 5-zone(0~4)에 매핑 — 카메라 조준과 침입 판정이 항상 같은 존을 가리킴
- **연속 관측 게이트**: 금지 존에서 `intrusion_min_frames`(기본 3) 연속 관측 시에만 확정 — 존 경계 각도 노이즈(±2~3°)로 1~2프레임 걸치는 오탐 차단
- **재알림 throttle**: 같은 사람은 `intrusion_repeat_ms`(기본 10초) 간격으로만 재알림, 사라진 사람 상태는 5초 후 자동 정리(`PruneStale`)
- **침입 확정 시**: ① 서보 즉시 조준·오버레이 사진 ② FastAPI `intrusion-event` ③ Unity/시각화기 `intrusion` 이벤트(UDP 손실 대비 5프레임 반복 전송) ④ 전후 10초 블랙박스 저장
- 구현: `include/zone_policy.h` (header-only) + `ZP_zone_policy_unit` 단위 테스트

### 4.6 불법 투기 감지 (핵심 기능)

**본 시스템의 핵심 기능이다.** 4개 탐지 경로(궤적 근접/폭 감소/클러스터 분열/정지 트랙 재매칭)와 12종 오탐 가드(source lock, birth-time gate, hotspot 차단 등)로 투기 행위를 확정하고, 금지구역(옵션) 내 투기는 `severity="high"`로 전송한다. 2D LiDAR의 구조적 한계상 오탐을 0으로 만들 수는 없으므로, LiDAR/카메라는 **1차 스크리닝**으로 후보를 잡고 **최종 판단은 관리자가 증거 사진으로 확인**하는 운영을 전제한다 (줄여야 하는 것은 오탐이 아니라 미탐).

> 탐지 경로·오탐 가드·현장 튜닝 절차·트러블슈팅 이력 전체는 [`docs/투기감지_확장_레퍼런스.md`](docs/투기감지_확장_레퍼런스.md) 참조.

#### 4.6.1 확정 후 재검증 (v55 — 오탐 전송 차단)

판정 FSM 을 더 조이는 대신, 확정 **뒤에** 물리적 사실 하나를 검증한다: *진짜 버려진 쓰레기는 확정 후에도 그 자리에 정지 상태로 남고, 반사/고스트/잔상은 수 초 안에 사라지거나 움직인다.*

- 확정 순간의 증거(서보 사진·dump bundle·블랙박스 요청)는 기존대로 **즉시** 확보 — 투기자가 떠나기 전에 찍어야 하므로 지연하지 않는다
- **서버 전송과 Unity `dumping` 이벤트만** 기본 30프레임(약 3초) 잔존 확인 후 내보낸다
- 통과 시 페이로드에 `confidence`(`"high"`: 재관측 ≥80% + 투기자 이탈 / `"medium"`)와 `validated_frames`/`validate_window` 필드 추가 → **관리자가 "얼마나 확실한 확정인지" 보고 검토 우선순위를 정할 수 있다** (additive 필드 — 기존 서버 호환)
- 취소 시 `[DUMP-CANCEL]` 로그만 남고 전송·표시되지 않는다 (사라짐 = 고스트/반사, 300mm 초과 이동 = 정지 투기물 아님). 로컬 증거 파일은 원인 분석용으로 유지
- 구현: `include/dump_validation.h` (header-only) + `DV_dump_validation_unit` 단위 테스트. dump_detector FSM·기존 회귀 시나리오 28종은 변경 없음

---

## 5. 데이터 형식 (Data Format)

> **전체 필드 명세는 [`보안/통신_명세.md`](보안/통신_명세.md)가 단일 출처다** — Unity/FastAPI로 보내는 모든 패킷·페이로드의 필드 표, 단위, 이벤트 흐름, 계획(v51) 스키마까지 정리되어 있다. 아래는 요약.

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
- `events[].type` = `"intrusion"`(금지구역 침입, v51) / `"dumping"`(투기 확정) / `"departure"`(퇴장·잔상 제거). intrusion/departure는 UDP 손실 대비 5프레임 반복 전송되므로 Unity는 중복 수신을 무시(멱등 처리)해야 함

금지구역 침입 이벤트 (v51, 보안 감시 핵심):
```json
{
    "type": "intrusion",
    "person_track_id": 5, "person_x": 1200.0, "person_y": -800.0,
    "zone": 3, "severity": "high",
    "timestamp": 1712290800000
}
```

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
rplidar_app # 프로덕션 바이너리
rplidar_sim # 회귀 테스트 (exit code = fail count)
check_lidar # USB 센서 테스트
check_camera # 카메라 테스트
check_pir # PIR 센서 테스트 (EKMC160111 x2, GPIO 17/27)
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

재컴파일 없이 환경변수로 추적/확정 값을 조정할 수 있다 (기본은 v36 값). 투기 감지 확장 기능의 단계별 튜닝 절차·예시 명령은 [`docs/투기감지_확장_레퍼런스.md`](docs/투기감지_확장_레퍼런스.md) §3~§4 참조.

### systemd 배포
```ini
[Service]
Type=simple
ExecStart=/usr/local/bin/rplidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event 192.168.20.102:5005
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
| DV | dump_validation_unit | - | 투기 확정 재검증 단위 테스트 (잔존→high/이탈, 고스트 취소, 이동 취소, source 잔류→medium, 그룹 매칭) | 전체 assertion PASS |

| OC | object_classifier_unit | - | 객체 분류 단위 테스트 (노이즈/사람그룹/사람대역/동물 2경로/정지물체/unknown 상향/저신뢰 하향/타입 매핑/비활성) | 전체 assertion PASS |
| FM | face_mask_unit | - | 얼굴 마스킹 **fail-closed 계약** 단위 테스트 (모드 파싱, 안전한 기본값, 검출기 없을 때 원본 미출력, 명시적 비활성 시 허용) | 전체 assertion PASS |
| EV | evidence_vault_unit | - | 증거 원본 암호화 보관 단위 테스트 (hex 왕복, 키 없음 fail-closed, AES-256-GCM 왕복, 평문 미잔존, **변조 탐지**, 열람 로그, 보존기한 파기) | 전체 assertion PASS |

**상태**: **33/33 PASS** (v56 verified on Mac, 2026-07-31)

> OC·EV는 OpenCV/하드웨어 없이 도는 순수 로직 테스트라 Mac에서 완전 검증된다.
> EV의 암호화 왕복·변조 탐지는 OpenSSL이 있을 때만 실행되고, 없으면 fail-closed 동작만 검증한다.

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

### 운영 코드 — 불법 투기 감지 시스템이 사용 (`rplidar_app`에 포함)

| 관심사 | 파일 |
|--------|------|
| **메인 루프** | `src/main.cpp` (LiDAR + Camera + PIR + 네트워크 통합) |
| **금지구역 침입 판정 (핵심)** | `include/zone_policy.h` (header-only, 5-zone 정책) |
| **객체 추적 (사람/객체)** | `src/cluster_tracker.cpp` (7-phase FSM) |
| **칼만 필터** | `include/kalman_filter.h` (header-only) |
| **LiDAR 인터페이스** | `src/rplidar_s2.cpp` |
| **클러스터링** | `src/scan_processor.cpp` (DBSCAN + grid), `src/background_filter.cpp` |
| **파라미터 단일 출처** | `include/production_params.h` |
| **이벤트 전송 (FastAPI)** | `src/event_notifier.cpp` (HTTP POST + retry + 파일 큐) |
| **Unity/시각화기 송신** | `src/json_packet.cpp` (UDP JSON — intrusion/dumping/departure), `src/udp_sender.cpp` (바이너리 옵션) |
| **PIR 센서** | `src/pir_sensor.cpp` (libgpiod + sysfs fallback) |
| **카메라 (증거 사진·블랙박스)** | `src/camera_module.cpp` (PIMPL + async) |
| **서보 카메라 조준** | `src/servo_controller.cpp`, `include/servo_zone.h` |
| **투기 판정 (확장 기능)** | `include/dump_detector.h` (header-only) |
| **성능 프로파일** | `include/phase_profiler.h` (CSV) |
| **배포/설정** | `deploy/ecowarden.service` (systemd), `setup.sh`, `arduino/servo_zone_controller/` |

### 운영 보조 도구 (디버그/점검용)

| 도구 | 용도 |
|------|------|
| `lidar_visualizer.py` | 실시간 시각화 + **수신 데이터 값 패널** (`d` 토글, 이벤트 로그 `c` 클리어, 콘솔에 이벤트 JSON 출력) |
| `test_fastapi.py` | FastAPI 서버 연결 점검 (`python3 test_fastapi.py [intrusion]`) |
| `pi_photo_server.py` | **증거 사진 HTTP 서버** — 백엔드 부재 시 Unity 가 기기에서 사진을 직접 받는 우회 경로 (§3.11) |
| `check_hardware.sh` | 하드웨어 일괄 점검 |

### 테스트 코드 — 제품 바이너리에 미포함, CMake 타깃이 참조 (삭제 금지)

| 파일 | 타깃 | 검증 대상 |
|------|------|----------|
| `test/sim_main.cpp` | `rplidar_sim` | 추적·침입·투기 회귀 시나리오 28종 + 단위 테스트 5종(존 정책·투기 재검증·객체 분류·증거 보관소) (§7) |
| `test/test_camera.cpp` | `check_camera` | USB 카메라 캡처 |
| `test/test_lidar.cpp` | `check_lidar` | RPLiDAR S2 USB 연결 |
| `test/test_pir.cpp` | `check_pir` | PIR 센서 GPIO 17/27 |
| `test/servo_sweep_test.cpp` | `servo_sweep_test` | 서보 5-zone sweep + 촬영 |
| `test/servo_only_test.cpp` | `servo_only_test` | Pi5 단독 PWM(GPIO18) 진단 |

> 같은 분류가 `.gitignore` 상단 주석에도 기록되어 있다 (생성물·레거시만 git 제외).

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

과거 현장 이슈 25건의 원인·해결 기록은 [`docs/투기감지_확장_레퍼런스.md`](docs/투기감지_확장_레퍼런스.md) §5로 이전했다 (보안 감시 전환 시 본편 정리). 버전별 상세 분석은 `PROJECT_STATE.md` 참조.

---

## 10. 대전 지역 적용 시나리오 (지속가능발전 연계)

본 시스템은 CCTV 단독 감시의 한계(24시간 상시 저장에 따른 저장공간 낭비·사생활 침해, 사후 확인 부담)를 **LiDAR 우선 감시 + 이벤트 시에만 카메라 증거**로 보완하는 불법 투기 감지 구조다. 유동 인구가 많은 일반 보행로가 아니라, **불법 투기가 잦은 사각지대**(원도심 골목·하천변·상가 후면·이면도로 등 사람 통행이 적은 지점)를 대상으로 하므로 오탐 설명이 쉽고 사생활 노출이 적다.

| 적용 대상 | 활용 방식 |
|---|---|
| 원도심 골목·상습 무단투기 지점 | 불법 투기 감지(§4.6)로 투기 순간 증거화 — 오버레이 사진 + 머리 위 ID 라벨 + 전후 10초 영상. 반복 투기 지점 데이터화 |
| 갑천·유등천·대전천 하천변 | 수변 무단 투기·폐기물 방치 감지로 하천 오염 예방. 야간 IR 카메라 + LiDAR로 조명 없는 구간 대응 |
| 상가 후면·이면도로 배출 위반 지점 | 종량제 미사용·무단 배출 증거 확보 |
| (확장·옵션) 공공기관 외곽·시설 보호구역 | 동일 하드웨어로 금지구역 침입 감지 모드(§3.7) 운용 — `intrusion` 이벤트 + 서보 조준 |

대전 지속가능발전목표(D-SDGs, 대전지속가능발전협의회 tjla21.or.kr) 연계

| D-SDGs 목표 | 연계 내용 |
|---|---|
| **목표 11 「안전하고 살기좋은 환경 조성」** (주) | 11-4 도시환경 부정적 요소 감소 — 불법 투기 적발·억제로 도시 미관·위생 개선. (옵션) 11-3 안전관리 역량 증대 — 보호구역 침입 감시 |
| **목표 12 「자원의 효율적 생산·소비 추구」** (주) | 12-2 폐기물 절대적 감소 — 무단 투기 감시로 불법 폐기물 발생·정화비용 저감, 반복 투기 지점 데이터화 |
| **목표 6 「건강하고 안전한 물관리」** (보조) | 3대 하천(갑천·유등천·대전천) 수변 무단 투기 감지로 수질·수변 생태 보호 |
| **목표 16 「포용적 제도 구축」** (보조) | 투기 이벤트를 시각·좌표·사진·영상과 함께 위·변조 불가(WORM) 체계로 기록 — 단속 행정의 투명성·신뢰성 보강 |

- **경제성**: 상주 인력·상시 녹화 대비 저비용 — 이벤트 중심 저장으로 저장공간 절감, Pi 5 엣지 처리로 서버 부하 최소화
- **사회적 수용성**: 평상시에는 비식별 LiDAR 점군만으로 감시하고, 침입/투기 의심 시에만 카메라 증거를 남기는 **사생활 보호형 설계**. 이벤트 없이 사람이 지나가면 임시 사진은 자동 삭제된다(§3.6)
- 발표 대본·상세 시나리오·통신 명세는 [`보안/`](보안/README.md) 폴더 참조
