/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - RplidarS2 LiDAR 센서 인터페이스
 */

/**
 * @file  rplidar_s2.h
 * @brief Slamtec RPLiDAR S2 센서 래퍼 클래스 (ToF 30m)
 * @date  2026
 *
 * Slamtec rplidar_sdk (sl) 기반 RplidarS2 제어 인터페이스.
 * 360도 ToF 스캔 데이터를 (각도, 거리, 강도) 벡터로 반환한다.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <memory>

#include "scan_types.h"

// Slamtec RPLidar SDK
#include "sl_lidar.h"

namespace ecowarden {

// ── 에러 코드 ────────────────────────────────────────────────────────
enum class Error {
    None,
    ConnectionFailed,  // 시리얼 포트 연결 실패
    CommTimeout,       // 통신 대기 타임아웃
    ScanTimeout,       // 스캔 데이터 수신 타임아웃
    AlreadyRunning,    // 이미 스캔 중
    NotRunning,        // 스캔이 시작되지 않음
    MotorError,        // 모터 가동 실패
    InvalidData        // 잘못된 데이터 형식
};

const char* ErrorToString(Error e);

// ── 설정 구조체 ──────────────────────────────────────────────────────
struct Config {
    std::string serial_port   = "/dev/ttyUSB0";
    uint32_t    baudrate      = 1000000;    // RPLiDAR S2: 1M bps
    int64_t     connect_timeout_ms = 3500;  // 연결 대기 타임아웃 (ms)
    int64_t     scan_timeout_ms    = 1500;  // 스캔 데이터 대기 타임아웃 (ms)
    bool        enable_filter = true;       // 센서 내부 필터 활성화 여부
};

// ── RplidarS2 LiDAR 래퍼 클래스 ───────────────────────────────────────────
class RplidarS2 {
public:
    RplidarS2();
    ~RplidarS2();

    RplidarS2(const RplidarS2&) = delete;
    RplidarS2& operator=(const RplidarS2&) = delete;

    /**
     * @brief 센서에 연결하고 연속 스캔을 시작한다.
     * @param cfg  설정 (기본값 사용 가능)
     * @return Error::None 성공, 그 외 에러 코드
     */
    Error Start(const Config& cfg = Config{});

    /**
     * @brief 스캔을 중지하고 시리얼 포트를 닫는다.
     * @return Error::None 성공
     */
    Error Stop();

    /**
     * @brief 한 프레임(360° 1회전)의 스캔 데이터를 가져온다.
     * @param[out] frame  스캔 포인트 벡터
     * @return Error::None 성공, Error::ScanTimeout 시간 초과
     */
    Error GetScanFrame(ScanFrame& frame);

    /**
     * @brief 현재 스캔 주파수(Hz)를 반환한다.
     */
    bool GetScanFrequency(double& hz);

    /**
     * @brief 센서가 정상 동작 중인지 확인한다.
     */
    bool IsRunning() const;

private:
    sl::ILidarDriver*  driver_ = nullptr;
    std::atomic<bool>  running_{false};
    Config             cfg_;

    static uint64_t GetTimestampNs();
};

} // namespace ecowarden
