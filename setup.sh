#!/usr/bin/env bash
# EcoWarden one-shot setup: packages, permissions, SDK, configure, build.
# Raspberry Pi OS / Ubuntu 22.04+ target.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/thirdparty/rplidar_sdk"
BUILD_DIR="${SCRIPT_DIR}/build"
REBOOT_REQUIRED=0
SERVO_BACKEND="${ECOWARDEN_SERVO_BACKEND:-arduino}"
IS_PI5=0

echo "=== EcoWarden Setup: dependencies + permissions + full build ==="

if ! command -v sudo >/dev/null 2>&1; then
    echo "[ERROR] sudo is required on Raspberry Pi/Ubuntu setup."
    exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
    echo "[ERROR] setup.sh currently supports apt-based systems only."
    exit 1
fi

echo "[1/8] Installing system packages..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gpiod \
    libcurl4-openssl-dev \
    libgpiod-dev \
    libopencv-dev \
    nlohmann-json3-dev \
    pkg-config \
    python3 \
    python3-pip \
    python3-pygame \
    python3-venv \
    v4l-utils

echo "[2/8] Verifying Python visualizer dependency..."
python3 -c "import pygame; print('   -> Pygame', pygame.ver, 'OK')" 2>/dev/null \
    || echo "   -> WARNING: pygame not available, lidar_visualizer.py will not work"

echo "[3/8] Setting user groups for LiDAR, camera, GPIO..."
if ! getent group gpio >/dev/null 2>&1; then
    sudo groupadd gpio
    echo "   -> Created gpio group"
fi
sudo usermod -aG dialout,video,gpio "$USER"
echo "   -> Added $USER to dialout, video, gpio"

echo "[4/8] Installing udev rules..."
# MODE 0660 + dialout: 모든 사용자 접근(0666) 대신 dialout 그룹만 허용
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0660", GROUP:="dialout", SYMLINK+="rplidar"' \
    | sudo tee /etc/udev/rules.d/99-rplidar.rules >/dev/null

cat << 'GPIORULE' | sudo tee /etc/udev/rules.d/99-gpio.rules >/dev/null
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport 2>/dev/null || true; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport 2>/dev/null || true'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/direction /sys%p/value 2>/dev/null || true; chmod 660 /sys%p/direction /sys%p/value 2>/dev/null || true'"
GPIORULE

sudo udevadm control --reload-rules
sudo udevadm trigger
echo "   -> /dev/rplidar and /dev/gpiochip* permissions configured"

echo "[5/8] Preparing Slamtec rplidar_sdk..."
mkdir -p "${SCRIPT_DIR}/thirdparty"
if [ ! -d "${SDK_DIR}/.git" ]; then
    rm -rf "${SDK_DIR}"
    git clone https://github.com/Slamtec/rplidar_sdk.git "${SDK_DIR}"
else
    git -C "${SDK_DIR}" fetch --depth=1 origin master || true
    git -C "${SDK_DIR}" pull --ff-only || true
fi

if [ ! -f "${SDK_DIR}/sdk/include/sl_lidar.h" ]; then
    echo "[ERROR] rplidar_sdk is present but sdk/include/sl_lidar.h is missing."
    echo "        Remove ${SDK_DIR} and rerun ./setup.sh."
    exit 1
fi
echo "   -> rplidar_sdk ready at ${SDK_DIR}"

echo "[6/8] Configuring servo backend..."
if [ "${SERVO_BACKEND}" = "arduino" ]; then
    echo "   -> Arduino USB serial backend selected (default)"
    echo "   -> Upload arduino/servo_zone_controller/servo_zone_controller.ino to Arduino first"
    echo "   -> Pi will send servo angles over /dev/ttyACM0 at 115200 baud"
else
    echo "   -> PWM backend selected; ensuring RPi PWM overlay"
BOOT_CONFIG=""
if [ -f /boot/firmware/config.txt ]; then
    BOOT_CONFIG="/boot/firmware/config.txt"
elif [ -f /boot/config.txt ]; then
    BOOT_CONFIG="/boot/config.txt"
fi

if [ -f /proc/device-tree/model ] && tr -d '\0' </proc/device-tree/model | grep -q 'Raspberry Pi 5'; then
    IS_PI5=1
fi

if [ -n "${BOOT_CONFIG}" ]; then
    if [ "${IS_PI5}" -eq 1 ]; then
        # Pi5(RP1): 옵션 없이 'pwm-pi5' 적용 시 pwmchip2 (npwm=2) 가 생기고
        #   pwmchip2/pwm0 = GPIO18 (물리 pin 12)
        #   pwmchip2/pwm1 = GPIO19 (물리 pin 35)
        # 가 PWM alt 함수로 안정적으로 잡힘. 옵션을 주는 변형은 커널 버전마다
        # 이름이 달라 무시되는 사례가 잦으므로 기본 라우팅을 사용.
        OVERLAY_LINE='dtoverlay=pwm-pi5'
        OVERLAY_REGEX='^[[:space:]]*dtoverlay=pwm-pi5([[:space:]]|,|$)'
        OVERLAY_NOTE='Pi5(RP1) PWM: pwmchip2/pwm0 = GPIO18 (물리 pin 12)'
    else
        OVERLAY_LINE='dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4'
        OVERLAY_REGEX='^[[:space:]]*dtoverlay=pwm-2chan'
        OVERLAY_NOTE='Pi4 이하: GPIO12 = pwmchip0 channel 0'
    fi

    # 이전 setup이 GPIO12 강제 옵션을 적은 라인이 있다면 제거 (Pi5에서만 무력화됨)
    if [ "${IS_PI5}" -eq 1 ] && grep -Eq '^[[:space:]]*dtoverlay=pwm-pi5,' "${BOOT_CONFIG}"; then
        sudo sed -i '/^[[:space:]]*dtoverlay=pwm-pi5,/d' "${BOOT_CONFIG}"
        echo "   -> Removed previous pwm-pi5 line with options (was likely ignored by kernel)"
        REBOOT_REQUIRED=1
    fi

    if grep -Eq "${OVERLAY_REGEX}" "${BOOT_CONFIG}"; then
        echo "   -> PWM overlay already enabled in ${BOOT_CONFIG}"
    else
        printf '\n# EcoWarden SG90 servo (%s)\n%s\n' "${OVERLAY_NOTE}" "${OVERLAY_LINE}" \
            | sudo tee -a "${BOOT_CONFIG}" >/dev/null
        REBOOT_REQUIRED=1
        echo "   -> Added '${OVERLAY_LINE}' to ${BOOT_CONFIG}"
        echo "   -> ${OVERLAY_NOTE}"
        echo "   -> Reboot is required before PWM signal is routed to the GPIO pin."
    fi

    # Pi5: dtparam=audio=on 이 GPIO18/19를 PWM-audio 로 선점할 수 있어 비활성화.
    # (Pi5는 아날로그 오디오 잭이 없으므로 끔으로써 잃는 기능 없음.)
    if [ "${IS_PI5}" -eq 1 ] && grep -Eq '^[[:space:]]*dtparam=audio=on' "${BOOT_CONFIG}"; then
        sudo sed -i 's|^[[:space:]]*dtparam=audio=on|# dtparam=audio=on  # disabled by EcoWarden setup: PWM(GPIO18/19) 선점 회피|' \
            "${BOOT_CONFIG}"
        REBOOT_REQUIRED=1
        echo "   -> Commented out 'dtparam=audio=on' in ${BOOT_CONFIG}"
        echo "      (Pi5에는 아날로그 오디오가 없으므로 안전. PWM(GPIO18/19) 선점을 막음.)"
    fi
else
    echo "   -> WARNING: boot config not found; cannot add PWM overlay"
fi
fi

echo "[7/8] Configuring and building every target..."
cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

echo "[8/8] Build artifact check..."
required_bins=(
    rplidar_app
    rplidar_sim
    check_lidar
    check_camera
    check_pir
    servo_sweep_test
    servo_only_test
)

for bin in "${required_bins[@]}"; do
    if [ ! -x "${BUILD_DIR}/${bin}" ]; then
        echo "[ERROR] Missing build artifact: ${BUILD_DIR}/${bin}"
        exit 1
    fi
    echo "   -> ${bin} OK"
done

echo ""
echo "=== Setup completed successfully ==="
echo ""

# 재부팅이 필요 없는 상태 (이미 overlay 적용됨) 라면 즉시 PWM 환경 sanity check
if [ "${SERVO_BACKEND}" = "pwm" ] && [ "${REBOOT_REQUIRED}" -eq 0 ] && [ "${IS_PI5}" -eq 1 ]; then
    echo "── PWM 환경 즉시 점검 ────────────────────────────────────"
    if [ -d /sys/class/pwm/pwmchip2 ]; then
        echo "   /sys/class/pwm/pwmchip2 존재 OK (npwm=$(cat /sys/class/pwm/pwmchip2/npwm 2>/dev/null || echo '?'))"
        echo "   서보 신호선을 BCM GPIO18 = 물리 pin 12 에 꽂고:"
        echo "     sudo ./build/servo_only_test"
    else
        echo "   /sys/class/pwm/pwmchip2 없음 → overlay 적용은 됐지만 재부팅 필요할 수 있음"
        echo "   현재 chip 목록: $(ls /sys/class/pwm/ 2>/dev/null | tr '\n' ' ')"
    fi
    echo ""
fi

if [ "${REBOOT_REQUIRED}" -eq 1 ]; then
    echo "[REBOOT REQUIRED] PWM overlay was modified."
    echo "Run: sudo reboot"
    echo ""
    echo "재부팅 후 Pi5 기준:"
    echo "  ls /sys/class/pwm/   →  pwmchip0  pwmchip1  pwmchip2 (4채널)"
    echo "  서보 신호선은 BCM GPIO18 = 물리 pin 12 에 연결"
    echo "  코드 기본값이 pwmchip2 channel 2 (=GPIO18) 이므로 별도 설정 불필요"
    echo ""
fi
echo "권한 적용을 위해 로그아웃 후 다시 로그인하세요."
echo "즉시 GPIO만 확인하려면 새 터미널에서 'newgrp gpio' 후 실행하세요."
echo ""
echo "실행:"
echo "  sudo ./build/rplidar_app"
echo "  sudo ./build/rplidar_app /dev/ttyUSB0 https://api.ecowarden.systems/api/dumping-event 192.168.20.173:5005"
echo ""
echo "검증:"
echo "  ./build/rplidar_sim"
echo "  ./build/check_lidar /dev/ttyUSB0"
echo "  ./build/check_camera"
echo "  ./build/check_pir"
echo "  sudo ./build/servo_sweep_test"
echo ""
echo "  Arduino serial 기본값:"
echo "    ECOWARDEN_SERVO_BACKEND=arduino"
echo "    ECOWARDEN_SERVO_SERIAL_DEVICE=/dev/ttyACM0"
echo ""
echo "  PWM fallback 필요 시:"
echo "    ECOWARDEN_SERVO_BACKEND=pwm ./setup.sh"
echo "    sudo ECOWARDEN_SERVO_BACKEND=pwm ./build/servo_only_test"
echo ""
echo "── SG90 + Arduino 서보 배선 ─────────────────────────"
echo "  Pi USB            → Arduino USB"
echo "  Arduino D9        → SG90 주황/노랑 Signal"
echo "  Arduino GND       → SG90 갈색/검정 GND"
echo "  Arduino 5V 또는 외부 5V+ → SG90 빨강 VCC"
echo "  ※ 외부 5V 사용 시: 외부 GND 와 Arduino GND 를 반드시 묶을 것"
echo ""
echo "시각화:"
echo "  python3 lidar_visualizer.py"
