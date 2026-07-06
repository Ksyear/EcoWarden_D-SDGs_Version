#!/bin/bash
# EcoWarden: 하드웨어 연결 상태 진단 스크립트 (Linux 전용)

echo "=== EcoWarden Hardware Diagnostic ==="

# 1. LiDAR 포트 확인
echo -n "[1/3] LiDAR Port (/dev/ttyUSB0 or /dev/rplidar): "
if [ -e /dev/ttyUSB0 ] || [ -e /dev/rplidar ]; then
    echo "OK"
    ls -l /dev/ttyUSB0 2>/dev/null || ls -l /dev/rplidar
else
    echo "FAILED (Not found)"
fi

# 2. 카메라 장치 확인
echo -n "[2/3] Camera Device (/dev/video0): "
if [ -e /dev/video0 ]; then
    echo "OK"
    if command -v v4l2-ctl >/dev/null; then
        v4l2-ctl --device=/dev/video0 --all | grep -E "Card type|Name"
    fi
else
    echo "FAILED (Not found)"
fi

# 3. OpenCV 설치 확인
echo -n "[3/3] OpenCV Library: "
if ldconfig -p | grep -i opencv >/dev/null; then
    echo "OK"
else
    echo "NOT FOUND (Camera module will be disabled)"
fi

echo "======================================"
echo "진단 완료. 모든 항목이 OK여야 정상 작동합니다."
