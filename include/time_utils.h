/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: 데이터 무결성 보증형 디지털 트윈 관제 플랫폼
 * Module : EMBEDDED - RplidarS2 LiDAR 센서 인터페이스 및 객체 탐지
 */

/**
 * @file  time_utils.h
 * @brief epoch ms / ISO 8601 / 현재 시각 헬퍼 (header-only)
 *
 * 이전에는 cluster_tracker.cpp / event_notifier.cpp / json_packet.cpp가
 * 동일한 NowMs·MsToIso8601을 제각기 복사해 두고 있었다. 본 헤더로 통일.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

namespace ecowarden {

// 현재 wall-clock 시각을 epoch milliseconds로 반환한다.
inline uint64_t NowMs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
        ).count()
    );
}

// epoch milliseconds → ISO 8601 UTC 문자열 ("YYYY-MM-DDTHH:MM:SS.mmmZ").
inline std::string MsToIso8601(uint64_t epoch_ms) {
    time_t sec = static_cast<time_t>(epoch_ms / 1000);
    uint32_t ms_part = static_cast<uint32_t>(epoch_ms % 1000);

    struct tm utc{};
    gmtime_r(&sec, &utc);

    char buf[64];
    std::snprintf(buf, sizeof(buf),
                  "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
                  utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                  utc.tm_hour, utc.tm_min, utc.tm_sec, ms_part);
    return buf;
}

// epoch milliseconds → "YYYY-MM-DD HH:MM:SS" UTC (API payload 전용).
inline std::string MsToApiTime(uint64_t epoch_ms) {
    time_t sec = static_cast<time_t>(epoch_ms / 1000);
    struct tm utc{};
    gmtime_r(&sec, &utc);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                  utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                  utc.tm_hour, utc.tm_min, utc.tm_sec);
    return buf;
}

} // namespace ecowarden
