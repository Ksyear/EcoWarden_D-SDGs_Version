/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: 데이터 무결성 보증형 디지털 트윈 관제 플랫폼
 * Module : EMBEDDED - 금지구역(Zone) 정책 / 침입 탐지
 */

/**
 * @file  zone_policy.h
 * @brief "버리면 안 되는 지역" 존 정책 — 금지 존에 사람이 머물면 intrusion
 *        이벤트를 만든다 (header-only).
 *
 * 존 번호는 servo_zone.h 의 5-zone(0~4, 중심각 0/45/90/135/180°)과 동일한
 * 각도 분할을 사용한다. 금지 존 목록은 ECOWARDEN_RESTRICTED_ZONES="3,4"
 * 형식으로 주입한다 (비어 있으면 정책 비활성 — 기존 동작과 동일).
 *
 * 오탐 방지:
 *   - intrusion_min_frames: 존 경계 각도 노이즈로 1~2프레임 금지 존에 걸치는
 *     경우를 걸러내기 위해 연속 관측 프레임을 요구한다.
 *   - intrusion_repeat_ms: 같은 사람이 금지 존에 계속 머물러도 이벤트를
 *     이 간격보다 자주 반복 생성하지 않는다.
 */

#pragma once

#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_map>

namespace ecowarden {

struct ZonePolicyParams {
    // 쉼표 구분 금지 존 목록 (예: "3,4"). 비어 있으면 정책 비활성.
    std::string restricted_zones;
    // 금지 존 연속 관측 프레임 수 — 이 값 이상일 때만 intrusion 확정.
    uint32_t    intrusion_min_frames = 3;
    // 같은 사람 재알림 최소 간격 (ms).
    uint32_t    intrusion_repeat_ms  = 10000;
    // 사람이 사라진 뒤 상태를 잊기까지의 시간 (ms).
    uint32_t    forget_after_ms      = 5000;
};

// ── 금지구역 침입 이벤트 ─────────────────────────────────────────────
struct IntrusionEvent {
    uint32_t person_track_id = 0;
    int      zone            = -1;
    double   person_x_mm     = 0.0;
    double   person_y_mm     = 0.0;
    uint64_t timestamp_ms    = 0;
};

class ZonePolicy {
public:
    explicit ZonePolicy(const ZonePolicyParams& params) : params_(params) {
        std::stringstream ss(params.restricted_zones);
        std::string token;
        while (std::getline(ss, token, ',')) {
            char* end = nullptr;
            const long zone = std::strtol(token.c_str(), &end, 10);
            if (end != token.c_str() && zone >= 0 && zone <= 31) {
                mask_ |= 1u << static_cast<uint32_t>(zone);
            }
        }
    }

    bool Enabled() const { return mask_ != 0; }

    bool IsRestricted(int zone) const {
        return zone >= 0 && zone <= 31 && ((mask_ >> zone) & 1u) != 0;
    }

    /**
     * @brief 사람 1명의 현재 존 관측을 갱신한다.
     * @return intrusion 이 새로 확정되면 true 를 반환하고 evt 를 채운다.
     */
    bool OnPersonZone(uint32_t person_id, int zone,
                      double x_mm, double y_mm,
                      uint64_t now_ms, IntrusionEvent* evt) {
        if (!Enabled()) return false;

        auto& st = states_[person_id];
        st.last_seen_ms = now_ms;

        if (!IsRestricted(zone)) {
            st.consecutive = 0;
            return false;
        }

        st.consecutive++;
        if (st.consecutive < params_.intrusion_min_frames) return false;
        if (st.last_event_ms != 0 &&
            now_ms < st.last_event_ms + params_.intrusion_repeat_ms) {
            return false;
        }

        st.last_event_ms = now_ms;
        if (evt) {
            evt->person_track_id = person_id;
            evt->zone            = zone;
            evt->person_x_mm     = x_mm;
            evt->person_y_mm     = y_mm;
            evt->timestamp_ms    = now_ms;
        }
        return true;
    }

    // 오래 안 보인 사람의 상태 제거 — 트랙 ID 재사용 시 stale 상태 방지.
    void PruneStale(uint64_t now_ms) {
        for (auto it = states_.begin(); it != states_.end();) {
            if (now_ms > it->second.last_seen_ms + params_.forget_after_ms) {
                it = states_.erase(it);
            } else {
                ++it;
            }
        }
    }

    size_t ActiveStates() const { return states_.size(); }

private:
    struct State {
        uint32_t consecutive   = 0;
        uint64_t last_event_ms = 0;
        uint64_t last_seen_ms  = 0;
    };

    ZonePolicyParams params_;
    uint32_t mask_ = 0;
    std::unordered_map<uint32_t, State> states_;
};

} // namespace ecowarden
