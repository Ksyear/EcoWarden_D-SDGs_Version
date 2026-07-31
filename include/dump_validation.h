/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 투기 확정 후 재검증 (post-confirmation validation)
 */

/**
 * @file  dump_validation.h
 * @brief 투기 확정 이벤트를 서버/Unity 로 내보내기 전에 "투기물이 실제로
 *        현장에 남아 있는지"를 N 프레임 동안 재관찰해 오탐을 걸러낸다
 *        (header-only).
 *
 * 배경:
 *   2D LiDAR 로 투기 "행위"를 판정하는 dump_detector 의 FSM 은 반사 spike,
 *   배경 잔상, 다리 분리 등으로 오탐이 구조적으로 남는다 (v13~v37 튜닝
 *   이력 참조). 판정 파라미터를 더 조이면 미탐이 늘어나는 줄다리기가
 *   반복되므로, 이 모듈은 판정 FSM 을 건드리지 않고 그 **뒤에** 물리적
 *   사실 하나만 검증한다:
 *
 *     "진짜 버려진 쓰레기는 확정 후에도 그 자리에 정지 상태로 남는다.
 *      반사/고스트/잔상은 수 초 안에 사라지거나 움직인다."
 *
 * 동작:
 *   - 확정 순간의 증거(서보 사진, 블랙박스 요청, dump bundle)는 기존대로
 *     즉시 확보한다 — 사람이 떠나기 전에 찍어야 하므로 지연 불가.
 *   - 서버 전송(SendDumping)과 Unity `dumping` 이벤트만 검증 통과 후로
 *     미룬다 (기본 30프레임 = 10Hz 기준 약 3초).
 *   - 통과: confidence("high"/"medium") + 재관측 프레임 수를 페이로드에
 *     추가해 전송 → 관리자가 "얼마나 확실한 확정인지" 판단 가능.
 *   - 취소: [DUMP-CANCEL] 로그만 남기고 전송하지 않는다. 이미 저장된
 *     로컬 증거 파일은 삭제하지 않는다 (원인 분석용).
 *
 * 비활성: ECOWARDEN_DUMP_VALIDATE=0 → 기존과 동일하게 확정 즉시 전송.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "cluster_tracker.h"

namespace ecowarden {

struct DumpValidationParams {
    // 재검증 활성 여부. false 면 확정 즉시 전송 (기존 동작).
    bool     enable             = true;
    // 확정 후 관찰 프레임 수 (10Hz 기준 30 = 약 3초).
    uint32_t validate_frames    = 30;
    // 관찰 중 투기물이 확정 위치에서 이 이상 벗어나면 취소 (mm).
    // 움직이는 "쓰레기"는 사람 다리/추적 오류일 가능성이 높다.
    double   max_move_mm        = 300.0;
    // 관찰 기간 중 최소 재관측 비율 — 미달 시 고스트/반사로 보고 취소.
    double   min_present_ratio  = 0.5;
    // confidence="high" 재관측 비율 기준 (source 이탈 동반 시).
    double   high_present_ratio = 0.8;
    // 투기 주체가 투기물에서 이 이상 멀어지면 "이탈"로 판정 (mm).
    double   source_departed_mm = 1200.0;
};

// ── 재검증을 통과한 투기 이벤트 ─────────────────────────────────────
struct ValidatedDump {
    DumpingEvent evt;                    // 원본 확정 이벤트 (시각 = 확정 시각)
    std::string  image_base64;           // 확정 순간 캡처 이미지
    int          zone            = -1;   // 서보 5-zone (0~4)
    bool         in_restricted   = false;
    bool         pir_motion      = false;
    // 검증 결과 — 서버 페이로드에 additive 필드로 실린다.
    const char*  confidence      = "medium"; // "high" | "medium"
    uint32_t     observed_frames = 0;    // 관찰 창에서 재관측된 프레임 수
    uint32_t     window_frames   = 0;    // 관찰 창 길이
    bool         source_departed = false; // 투기 주체 이탈 여부
};

class DumpValidator {
public:
    explicit DumpValidator(const DumpValidationParams& params)
        : params_(params) {}

    bool Enabled() const { return params_.enable; }
    size_t PendingCount() const { return pending_.size(); }

    /**
     * @brief 투기 확정 이벤트를 재검증 대기열에 넣는다.
     *        같은 object_track_id 가 이미 대기 중이면 무시한다
     *        (tracker 의 dump_alert_fired 가 재확정을 막지만 방어적으로).
     */
    void OnDumpConfirmed(const DumpingEvent& evt, std::string image_base64,
                         int zone, bool in_restricted, bool pir_motion,
                         uint32_t frame) {
        for (const auto& pd : pending_) {
            if (pd.out.evt.object_track_id == evt.object_track_id) return;
        }
        Pending pd;
        pd.out.evt           = evt;
        pd.out.image_base64  = std::move(image_base64);
        pd.out.zone          = zone;
        pd.out.in_restricted = in_restricted;
        pd.out.pir_motion    = pir_motion;
        pd.start_frame       = frame;
        pending_.push_back(std::move(pd));
    }

    /**
     * @brief 매 프레임 호출. 관찰이 끝나 검증을 통과한 이벤트를 out 에
     *        append 하고, 취소된 이벤트는 로그만 남긴다.
     * @param tracks 현재 프레임의 트랙 목록 (tracker.GetTracks()).
     * @param frame  현재 프레임 번호 (main loop 의 frame_count).
     */
    /**
     * @param cancelled_object_ids (선택) 재검증에서 **취소된** 투기물 트랙 ID.
     *        호출부가 `ClusterTracker::ClearDumpFlags()` 로 되돌려 줘야 한다.
     *
     *   왜 여기서 직접 안 고치나: 검증기는 트랙을 소유하지 않는다(const 참조).
     *   플래그를 되돌리는 책임은 트랙을 소유한 트래커에 둔다.
     *
     *   되돌리지 않으면 취소된 오탐이 `is_dumped_item=true` 인 채로 남아
     *   Unity 화면에 최대 `dumped_item_lost_age_limit`(기본 200프레임 = 20초)
     *   동안 쓰레기로 계속 표시된다 — 전송만 막고 화면은 못 막는 반쪽 취소.
     */
    void Update(const std::vector<Track>& tracks, uint32_t frame,
                std::vector<ValidatedDump>* out,
                std::vector<uint32_t>* cancelled_object_ids = nullptr) {
        for (auto it = pending_.begin(); it != pending_.end();) {
            Pending& pd = *it;

            // 투기물 / 투기 주체 트랙 탐색. person_track_id 는 그룹 ID 일
            // 수 있으므로 (v33) track id 와 person_group_id 양쪽을 본다.
            const Track* obj = nullptr;
            const Track* src = nullptr;
            for (const auto& t : tracks) {
                if (t.id == pd.out.evt.object_track_id) obj = &t;
                if (t.id == pd.out.evt.person_track_id ||
                    (t.person_group_id != 0 &&
                     t.person_group_id == pd.out.evt.person_track_id)) {
                    if (t.lost_count == 0) src = &t;
                }
            }

            bool moved = false;
            if (obj && obj->lost_count == 0) {
                const double dx = obj->x_mm - pd.out.evt.object_x_mm;
                const double dy = obj->y_mm - pd.out.evt.object_y_mm;
                if (std::sqrt(dx * dx + dy * dy) > params_.max_move_mm) {
                    moved = true;
                } else {
                    pd.seen++;
                }
            }

            // 투기 주체 이탈: 시야에서 사라졌거나 투기물에서 충분히 멀어짐.
            // 한 번 이탈로 판정되면 유지한다 (재접근해도 투기 사실은 불변).
            if (!pd.out.source_departed) {
                if (!src) {
                    pd.out.source_departed = true;
                } else {
                    const double dx = src->x_mm - pd.out.evt.object_x_mm;
                    const double dy = src->y_mm - pd.out.evt.object_y_mm;
                    if (std::sqrt(dx * dx + dy * dy) >
                        params_.source_departed_mm) {
                        pd.out.source_departed = true;
                    }
                }
            }

            if (moved) {
                std::printf("[DUMP-CANCEL] object %u — 확정 후 %.0fmm 초과"
                            " 이동, 정지 투기물 아님 (오탐 차단)\n",
                            pd.out.evt.object_track_id, params_.max_move_mm);
                if (cancelled_object_ids) {
                    cancelled_object_ids->push_back(pd.out.evt.object_track_id);
                }
                it = pending_.erase(it);
                continue;
            }

            const uint32_t elapsed = frame - pd.start_frame;
            if (elapsed >= params_.validate_frames) {
                const double ratio =
                    static_cast<double>(pd.seen) /
                    static_cast<double>(params_.validate_frames);
                if (ratio >= params_.min_present_ratio) {
                    pd.out.observed_frames = pd.seen;
                    pd.out.window_frames   = params_.validate_frames;
                    pd.out.confidence =
                        (ratio >= params_.high_present_ratio &&
                         pd.out.source_departed)
                            ? "high"
                            : "medium";
                    std::printf("[DUMP-VALID] object %u — %u/%u 프레임 잔존,"
                                " source %s → confidence=%s\n",
                                pd.out.evt.object_track_id, pd.seen,
                                params_.validate_frames,
                                pd.out.source_departed ? "이탈" : "잔류",
                                pd.out.confidence);
                    out->push_back(std::move(pd.out));
                } else {
                    std::printf("[DUMP-CANCEL] object %u — 관찰 %u프레임 중"
                                " %u프레임만 재관측, 고스트/반사 추정"
                                " (오탐 차단)\n",
                                pd.out.evt.object_track_id,
                                params_.validate_frames, pd.seen);
                    if (cancelled_object_ids) {
                        cancelled_object_ids->push_back(
                            pd.out.evt.object_track_id);
                    }
                }
                it = pending_.erase(it);
                continue;
            }
            ++it;
        }
    }

private:
    struct Pending {
        ValidatedDump out;
        uint32_t      start_frame = 0;
        uint32_t      seen        = 0;  // 확정 위치 근방 재관측 프레임 수
    };

    DumpValidationParams  params_;
    std::vector<Pending>  pending_;
};

}  // namespace ecowarden
