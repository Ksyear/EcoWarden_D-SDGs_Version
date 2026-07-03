/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: 데이터 무결성 보증형 디지털 트윈 관제 플랫폼
 * Module : EMBEDDED - RplidarS2 LiDAR 센서 인터페이스
 */

#include "rplidar_s2.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>

namespace ecowarden {

const char* ErrorToString(Error e) {
    switch (e) {
        case Error::None:             return "None";
        case Error::ConnectionFailed: return "ConnectionFailed";
        case Error::CommTimeout:      return "CommTimeout";
        case Error::ScanTimeout:      return "ScanTimeout";
        case Error::AlreadyRunning:   return "AlreadyRunning";
        case Error::NotRunning:       return "NotRunning";
        case Error::MotorError:       return "MotorError";
        case Error::InvalidData:      return "InvalidData";
        default:                      return "Unknown";
    }
}

RplidarS2::RplidarS2() : driver_(nullptr), running_(false) {
}

RplidarS2::~RplidarS2() {
    Stop();
}

Error RplidarS2::Start(const Config& cfg) {
    if (running_) return Error::AlreadyRunning;
    cfg_ = cfg;

    // 1. 드라이버 생성
    auto drv_res = sl::createLidarDriver();
    if (!drv_res) return Error::ConnectionFailed;
    driver_ = *drv_res;

    // 2. 연결
    auto chan_res = sl::createSerialPortChannel(cfg_.serial_port, cfg_.baudrate);
    if (!chan_res) {
        delete driver_;
        driver_ = nullptr;
        return Error::ConnectionFailed;
    }
    sl::IChannel* channel = *chan_res;

    if (SL_IS_FAIL(driver_->connect(channel))) {
        delete driver_;
        driver_ = nullptr;
        return Error::ConnectionFailed;
    }

    // 3. 센서 정보 확인 (Health Check)
    sl_lidar_response_device_health_t health;
    if (SL_IS_FAIL(driver_->getHealth(health))) {
        Stop();
        return Error::CommTimeout;
    }

    if (health.status == SL_LIDAR_STATUS_ERROR) {
        std::cerr << "[RPLIDAR] Internal error detected. Resetting..." << std::endl;
        driver_->reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    // 4. 모터 시작
    driver_->setMotorSpeed(); // Default speed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 5. 스캔 시작 (HQ Scan mode for S2)
    if (SL_IS_FAIL(driver_->startScan(0, 1))) {
        Stop();
        return Error::MotorError;
    }

    running_ = true;
    return Error::None;
}

Error RplidarS2::Stop() {
    if (driver_) {
        driver_->stop();
        driver_->setMotorSpeed(0);
        delete driver_;
        driver_ = nullptr;
    }
    running_ = false;
    return Error::None;
}

Error RplidarS2::GetScanFrame(ScanFrame& frame) {
    if (!running_ || !driver_) return Error::NotRunning;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    sl_result op_result = driver_->grabScanDataHq(nodes, count);
    if (SL_IS_FAIL(op_result)) {
        return Error::ScanTimeout;
    }

    // 데이터 오름차순 정렬 (각도 기준)
    driver_->ascendScanData(nodes, count);

    frame.clear();
    frame.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        // 유효한 데이터만 추출 (거리 > 0)
        if (nodes[i].dist_mm_q2 > 0) {
            ScanPoint pt;
            // angle_z_q14 is in degree * 64, dist_mm_q2 is in mm * 4
            pt.angle_deg = nodes[i].angle_z_q14 * 90.f / 16384.f;
            pt.distance_mm = static_cast<uint16_t>(nodes[i].dist_mm_q2 / 4.0f);
            pt.intensity = static_cast<uint8_t>(nodes[i].quality >> 2);
            frame.push_back(pt);
        }
    }

    return Error::None;
}

bool RplidarS2::GetScanFrequency(double& hz) {
    if (!running_ || !driver_) return false;
    
    // Simplified frequency check
    hz = 10.0; // RPLiDAR S2 default
    return true;
}

bool RplidarS2::IsRunning() const {
    return running_;
}

uint64_t RplidarS2::GetTimestampNs() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace ecowarden
