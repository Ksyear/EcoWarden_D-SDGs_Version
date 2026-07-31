/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - PIR 센서 인터페이스 (libgpiod + sysfs fallback)
 */

#pragma once

#include <cstdint>
#include <string>

#ifdef USE_GPIOD
struct gpiod_chip;
struct gpiod_line;
#endif

namespace ecowarden {

struct PirSensorConfig {
    uint32_t gpio_pin          = 17;
    bool     active_high       = true;
    uint32_t debounce_frames   = 3;
    uint32_t holdoff_frames    = 50;
    const char* gpiod_chip     = "gpiochip4";  // RPi5 RP1 GPIO chip
};

class PirSensor {
public:
    explicit PirSensor(const PirSensorConfig& config = PirSensorConfig{});
    ~PirSensor();

    PirSensor(const PirSensor&)            = delete;
    PirSensor& operator=(const PirSensor&) = delete;

    bool Init();
    void Read();
    bool IsMotionDetected() const { return motion_active_; }
    bool IsReady() const { return initialized_; }
    bool HasStateChanged() const { return state_changed_; }

private:
    PirSensorConfig config_;
    bool            initialized_   = false;
    bool            motion_active_ = false;
    bool            state_changed_ = false;
    bool            raw_state_     = false;
    uint32_t        stable_count_  = 0;
    uint32_t        holdoff_remaining_ = 0;

    // sysfs fallback
    std::string     value_path_;

    // libgpiod
#ifdef USE_GPIOD
    gpiod_chip* chip_ = nullptr;
    gpiod_line* line_ = nullptr;
#endif
    bool use_gpiod_ = false;

    bool InitGpiod();
    bool InitSysfs();
    bool ReadGpioRaw();
};

} // namespace ecowarden
