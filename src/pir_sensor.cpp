/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - PIR 센서 인터페이스 (libgpiod + sysfs fallback)
 */

#include "pir_sensor.h"
#include <cstdio>

#ifdef __linux__
#include <fstream>
#include <sstream>
#include <unistd.h>
#endif

#ifdef USE_GPIOD
#include <gpiod.h>
#endif

namespace ecowarden {

PirSensor::PirSensor(const PirSensorConfig& config)
    : config_(config) {}

PirSensor::~PirSensor() {
#ifdef USE_GPIOD
    if (use_gpiod_) {
        if (line_) gpiod_line_release(line_);
        if (chip_) gpiod_chip_close(chip_);
        return;
    }
#endif
#ifdef __linux__
    if (initialized_ && !use_gpiod_) {
        std::ofstream unexport_fs("/sys/class/gpio/unexport");
        if (unexport_fs.is_open()) {
            unexport_fs << config_.gpio_pin;
        }
    }
#endif
}

bool PirSensor::Init() {
#ifdef __linux__
    // 1차: libgpiod 시도 (RPi5 + 최신 커널 호환)
    if (InitGpiod()) {
        std::printf("[PIR] GPIO %u initialized via libgpiod (%s)\n",
                    config_.gpio_pin, config_.gpiod_chip);
        return true;
    }

    // 2차: sysfs fallback (레거시 RPi3/4)
    if (InitSysfs()) {
        std::printf("[PIR] GPIO %u initialized via sysfs (legacy)\n",
                    config_.gpio_pin);
        return true;
    }

    std::fprintf(stderr, "[PIR] GPIO %u init failed.\n"
                 "  - libgpiod: apt install libgpiod-dev && cmake with -DUSE_GPIOD=ON\n"
                 "  - sysfs: sudo 또는 gpio 그룹 권한 확인\n"
                 "  - RPi5: gpiochip4 사용 (RP1), RPi4: gpiochip0\n",
                 config_.gpio_pin);
    return false;
#else
    initialized_   = true;
    motion_active_ = true;
    return true;
#endif
}

bool PirSensor::InitGpiod() {
#ifdef USE_GPIOD
    const char* chips[] = {
        config_.gpiod_chip,
        "gpiochip4",
        "gpiochip0",
        "gpiochip1",
        "gpiochip2",
        "gpiochip3",
        "gpiochip5",
    };

    for (const char* chip_name : chips) {
        if (!chip_name || chip_name[0] == '\0') continue;
        chip_ = gpiod_chip_open_by_name(chip_name);
        if (!chip_) continue;

        line_ = gpiod_chip_get_line(chip_, config_.gpio_pin);
        if (line_) {
            break;
        }
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
    if (!chip_) return false;
    if (!line_) return false;

    int flags = config_.active_high ? 0 : GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW;
    struct gpiod_line_request_config req_cfg = {
        .consumer = "ecowarden-pir",
        .request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT,
        .flags = flags,
    };

    if (gpiod_line_request(line_, &req_cfg, 0) < 0) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
        return false;
    }

    use_gpiod_ = true;
    initialized_ = true;
    return true;
#else
    return false;
#endif
}

bool PirSensor::InitSysfs() {
#ifdef __linux__
    std::ostringstream oss;
    oss << "/sys/class/gpio/gpio" << config_.gpio_pin;
    std::string gpio_dir = oss.str();
    value_path_ = gpio_dir + "/value";

    {
        std::ifstream check(gpio_dir + "/direction");
        if (!check.is_open()) {
            std::ofstream export_fs("/sys/class/gpio/export");
            if (!export_fs.is_open()) return false;
            export_fs << config_.gpio_pin;
            export_fs.close();
            usleep(100000);
        }
    }

    std::ofstream dir_fs(gpio_dir + "/direction");
    if (!dir_fs.is_open()) return false;
    dir_fs << "in";

    use_gpiod_ = false;
    initialized_ = true;
    return true;
#else
    return false;
#endif
}

bool PirSensor::ReadGpioRaw() {
#ifdef USE_GPIOD
    if (use_gpiod_ && line_) {
        int val = gpiod_line_get_value(line_);
        if (val < 0) return false;
        // active_low 처리는 libgpiod 플래그로 이미 적용됨
        raw_state_ = (val == 1);
        return true;
    }
#endif
#ifdef __linux__
    if (!use_gpiod_) {
        std::ifstream val_fs(value_path_);
        if (!val_fs.is_open()) return false;
        char ch = '0';
        val_fs >> ch;
        bool pin_high = (ch == '1');
        raw_state_ = config_.active_high ? pin_high : !pin_high;
        return true;
    }
#endif
#ifndef __linux__
    raw_state_ = true;
#endif
    return true;
}

void PirSensor::Read() {
    state_changed_ = false;
#ifndef __linux__
    motion_active_ = true; return;
#endif
    if (!initialized_) return;
    if (!ReadGpioRaw()) return;

    bool prev_active = motion_active_;
    if (holdoff_remaining_ > 0) holdoff_remaining_--;

    if (raw_state_ != motion_active_) {
        stable_count_++;
        if (stable_count_ >= config_.debounce_frames) {
            if (raw_state_) {
                motion_active_ = true;
                holdoff_remaining_ = config_.holdoff_frames;
            } else if (holdoff_remaining_ == 0) {
                motion_active_ = false;
            }
            stable_count_ = 0;
        }
    } else {
        stable_count_ = 0;
    }

    if (raw_state_ && motion_active_) holdoff_remaining_ = config_.holdoff_frames;
    state_changed_ = (motion_active_ != prev_active);
}

} // namespace ecowarden
