# Ecowarden 배포 (deploy/)

v5 에서 신설. 이전에는 `.service` 파일이 레포에 0 개였고, "`ECOWARDEN_DRY_RUN`
의 운영 기본값이 무엇인가" 가 코드에만 존재했다. v5 에서 systemd unit 템플릿을
두어 정책을 명시한다.

## 파일

- `ecowarden.service` — systemd unit 템플릿 (자동 시작, 장애 시 재시작)

## systemd 설치

1. 바이너리를 `/opt/ecowarden/` 로 복사하고 소유자를 `ecowarden` 으로.
   ```
   sudo install -d -o ecowarden /opt/ecowarden /opt/ecowarden/captures /var/log/ecowarden
   sudo install -o ecowarden -m 755 build/rplidar_app /opt/ecowarden/
   ```
2. unit 파일 복사:
   ```
   sudo cp deploy/ecowarden.service /etc/systemd/system/
   ```
3. 환경에 맞춰 `WorkingDirectory` / `ExecStart` 의 endpoint URL / serial port 경로
   수정.
4. 활성화:
   ```
   sudo systemctl daemon-reload
   sudo systemctl enable --now ecowarden
   sudo systemctl status ecowarden
   journalctl -u ecowarden -f
   ```

## 환경 변수 정책 (v5)

### `ECOWARDEN_DRY_RUN`

- `0` (운영 기본) — HTTP POST 실전송.
- `1` — 모든 이벤트 drop, `EventNotifier::dropped_count_` 만 증가.

v4 까지 이 값의 "운영 의도" 가 어디에도 문서화되지 않아서 혼동을 유발했다.
v5 의 `ecowarden.service` 는 `Environment=ECOWARDEN_DRY_RUN=0` 을 **명시**한다.
리허설은 복사본 unit (`ecowarden-dry.service`) 에서 `1` 로 바꾸어 구동.

### `ECOWARDEN_PROFILE_CSV`

- 설정 시 매 프레임 phase latency (us) + KF uncertainty 를 CSV 로 기록.
- 기본 OFF (디스크 IO 부담). 성능 디버깅이 필요할 때만 일시적으로 enable.
- 사용 예:
  ```
  sudo systemctl edit ecowarden
  # [Service]
  # Environment=ECOWARDEN_PROFILE_CSV=/var/log/ecowarden/phases.csv
  sudo systemctl restart ecowarden
  ```
- 분석:
  ```
  # 100 frame 단위로 p50/p95 확인 — 런타임 stderr 에도 자동 출력
  journalctl -u ecowarden | grep PROF
  # 또는 CSV 를 pandas/Excel 에서 읽어 시계열 분석
  ```

## 회귀 게이트 (v5)

운영 배포 전, 본 레포 루트에서 아래를 통과해야 한다:

```
cd build && cmake --build . -j4
./rplidar_sim
# 기대: SUMMARY: 4 / 4 PASS, exit code 0
```

sim 회귀 스위트가 실패하면 배포 금지. 4/4 통과가 v5 머지 게이트.

## 검증

기계가 있다면:

```
sudo systemd-analyze verify deploy/ecowarden.service
```

없으면 형식 검토만 수행하고 실배포 환경에서 재확인.
