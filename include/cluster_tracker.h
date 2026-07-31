/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - RplidarS2 LiDAR 센서 인터페이스 및 객체 탐지
 */

/**
 * @file  cluster_tracker.h
 * @brief 프레임 간 클러스터 추적 및 객체 이탈(departure) 감지
 * @date  2026
 *
 * 상태 머신 (트랙 단위):
 *
 *   ┌──────────┐   이동거리 ≥ 50mm   ┌──────────┐
 *   │  MOVING  │◄────────────────────│STATIONARY│
 *   └────┬─────┘                     └────┬─────┘
 *        │  이동거리 < 50mm               │ 정지 3프레임 연속
 *        ▼                                ▼
 *   ┌──────────┐                     ┌──────────┐
 *   │STATIONARY│                     │ DEPARTED │ ← 이벤트 1회 발화
 *   └──────────┘                     └──────────┘
 *
 *   LOST: 매칭 실패 시 age_limit 초과 후 트랙 삭제
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>
#include <deque>
#include <utility>
#include <chrono>
#include <functional>
#include <string>

#include "scan_processor.h"
#include "kalman_filter.h"

namespace ecowarden {

// ── 트랙 상태 ────────────────────────────────────────────────────────
enum class TrackState {
    Moving = 0,      // 이동 중 (centroid 이동 ≥ threshold)
    Stationary = 1,  // 정지 중 (centroid 이동 < threshold)
    Departed = 2,    // 이탈 판정 완료 (이벤트 발화 후 전이)
    Lost = 3,        // 매칭 실패 — 삭제 대기
    Tentative = 4,   // 검증 중 — Unity 송신 보류
};

const char* TrackStateToString(TrackState s);

enum class TrackBirthSource {
    Unknown = 0,
    NormalForeground,
    SeparationDirect,
    BackgroundChange,
    StaticBgEdgeSpike,
    Recovery,
};

// ── 이탈 이벤트 ──────────────────────────────────────────────────────
struct DepartureEvent {
    uint32_t track_id;         // 추적 ID
    uint64_t timestamp_ms;     // 이탈 판정 시각 (epoch ms)
    double   x_mm;             // 이탈 위치 X (mm)
    double   y_mm;             // 이탈 위치 Y (mm)
    uint32_t stationary_frames;// 정지 유지 프레임 수
};

// ── 투기 이벤트 ──────────────────────────────────────────────────────
struct DumpingEvent {
    uint32_t person_track_id;  // 투기 주체 트랙 ID
    double   person_x_mm;      // 투기 주체 위치 X
    double   person_y_mm;      // 투기 주체 위치 Y
    double   person_cumulative_dist_mm; // 투기 주체 누적 이동거리
    uint32_t object_track_id;  // 투기물 트랙 ID
    double   object_x_mm;      // 투기물 위치 X
    double   object_y_mm;      // 투기물 위치 Y
    uint64_t timestamp_ms;     // 확정 시각 (epoch ms)

    // 증거 사진의 captures/ 기준 상대 경로 (v56).
    //   예: "dumps/evt_123/confirm.jpg"
    //   트래커는 이 값을 채우지 않는다 — main 이 촬영 후 채워 넣는다.
    //   Unity 는 이 경로로 정확한 사진을 집는다 (없으면 "최신 사진" 폴백).
    std::string image_file;
};

// ── 투기 의심 이벤트 (즉각 캡처용) ───────────────────────────────────
struct DumpingSuspectEvent {
    uint32_t person_track_id;
    uint32_t object_track_id;
    double   object_x_mm;
    double   object_y_mm;
    uint64_t timestamp_ms;
};

// ── 개별 트랙 ────────────────────────────────────────────────────────
struct Track {
    uint32_t   id;
    TrackState state;
    double     x_mm;              // 현재 centroid X
    double     y_mm;              // 현재 centroid Y
    double     prev_x_mm;         // 이전 프레임 centroid X
    double     prev_y_mm;         // 이전 프레임 centroid Y
    double     width_mm = 0.0;    // 클러스터 폭
    uint32_t   stationary_count;  // 연속 정지 프레임 수
    uint32_t   lost_count;        // 연속 매칭 실패 프레임 수
    uint32_t   age;               // 총 생존 프레임 수
    bool       was_moving;        // 정지 전에 이동 이력이 있었는지
    bool       departure_fired;   // 이탈 이벤트를 이미 발화했는지
    uint32_t   consecutive_match_count = 0; // 연속 매칭 성공 프레임 수
    uint32_t   total_match_count = 0;       // 누적 매칭 성공 프레임 수
    bool       confirmed = false;           // 송신 가능한 신뢰 트랙 여부
    uint32_t   person_group_id = 0;          // 0: 그룹 없음, 그 외 사람 그룹 ID
    bool       person_group_primary = false; // Unity 송신 대표 트랙 여부
    double     person_group_x_mm = 0.0;      // 그룹 중심 X
    double     person_group_y_mm = 0.0;      // 그룹 중심 Y
    double     person_group_width_mm = 0.0;  // 그룹 폭
    uint32_t   last_person_group_id = 0;      // group hold 용 직전 그룹 ID
    uint32_t   person_group_hold_count = 0;   // 일시적 분리 시 group 유지 잔여 프레임
    bool       is_object_candidate = false;  // 사람 그룹에서 제외할 정지 소형 객체 후보

    // ── 투기 감지 관련 필드 ──────────────────────────────────────────
    double     cumulative_dist_mm = 0.0;  // 누적 이동거리 (mm)
    bool       is_dump_suspect    = false; // 투기 의심 후보 (확정 전)
    bool       is_dumped_item     = false; // 투기 의심 물체 확정
    int        source_id          = -1;    // 투기 주체 트랙 ID (-1: 해당없음)
    double     source_x_mm        = 0.0;  // 분리 시점의 투기 주체 위치 X
    double     source_y_mm        = 0.0;  // 분리 시점의 투기 주체 위치 Y
    double     source_cumulative_dist_mm = 0.0; // 분리 시점의 투기 주체 누적 거리
    bool       dump_alert_fired   = false; // 투기 확정 알림 발화 여부
    bool       suspect_fired      = false; // 의심 알림(캡처) 발화 여부
    uint32_t   suspect_confirm_count = 0;  // 분리 후 독립 존재 프레임 카운터
    std::string suspect_photo_path;        // 의심 시점 캡처 사진 경로 (미확정 시 삭제)
    double     suspect_width_sum    = 0.0; // 의심 기간 중 폭 합계 (크기 일관성 검증용)
    double     suspect_width_sq_sum = 0.0; // 의심 기간 중 폭 제곱 합계 (분산 계산용)
    double     source_last_dist_mm  = 0.0; // source-person 과의 직전 거리
    uint32_t   source_dist_increase_count = 0; // source 와의 거리 증가 연속 프레임
    uint32_t   first_seen_frame = 0;        // 최초 관측 프레임 (birth-time gate)
    uint32_t   stationary_since_frame = 0;  // 정지 상태 시작 프레임
    uint32_t   source_bind_frame = 0;       // source lock 시점
    bool       source_locked = false;       // source ownership lock 여부
    uint32_t   source_person_entity_id = 0; // source PersonEntity/group id
    bool       newly_created_after_source = false; // source 접근 이후 생성된 물체인지
    bool       direct_dump_evidence = false; // width_drop/split/unmatched separation 직접 증거
    TrackBirthSource birth_source = TrackBirthSource::Unknown; // 생성 provenance
    std::deque<double> recent_move_history;  // 최근 이동량 평균으로 봉투 안정성 판단

    // -- 궤적 이력 (투기 분리 감지 강화) --
    std::deque<std::pair<double,double>> position_history;  // 최근 N 프레임 위치

    // -- 폭 변화 추적 (투기 물체 분리 보조 신호) --
    double     prev_width_mm        = 0.0;   // 이전 프레임 클러스터 폭
    bool       width_drop_detected  = false; // 폭 급감 이벤트 감지 플래그
    double     width_drop_x_mm      = 0.0;   // 폭 감소 발생 시점 위치 X
    double     width_drop_y_mm      = 0.0;   // 폭 감소 발생 시점 위치 Y

    // -- 속도 벡터 추적 (투사 궤적 분석) --
    double     vx_mm                = 0.0;   // 현재 프레임 X 속도 (mm/frame)
    double     vy_mm                = 0.0;   // 현재 프레임 Y 속도 (mm/frame)

    // -- 클러스터 분열 감지 --
    bool       split_detected       = false; // 이 트랙에서 클러스터 분열이 발생했는지
    double     split_x_mm           = 0.0;   // 분열 위치 X
    double     split_y_mm           = 0.0;   // 분열 위치 Y

    // -- 칼만 필터 (궤적 예측) --
    KalmanFilter2D kf;
};

inline bool IsUngroupedStationaryLargeObject(const Track& tr) {
    return !tr.is_dump_suspect &&
           !tr.is_dumped_item &&
           tr.person_group_id == 0 &&
           tr.width_mm >= 350.0 &&
           tr.stationary_count >= 2 &&
           tr.cumulative_dist_mm <= 200.0 &&
           !tr.was_moving;
}

inline bool IsBackgroundResidualTrack(const Track& tr) {
    return !tr.is_dump_suspect &&
           !tr.is_dumped_item &&
           tr.person_group_id == 0 &&
           tr.source_id < 0 &&
           tr.confirmed &&
           tr.lost_count == 0 &&
           tr.stationary_count >= 4 &&
           tr.total_match_count >= 4 &&
           tr.width_mm >= 250.0 &&
           tr.width_mm <= 350.0 &&
           tr.cumulative_dist_mm <= 120.0 &&
           !tr.was_moving;
}

inline bool IsHeldPersonLeg(const Track& tr) {
    return !tr.is_dump_suspect &&
           !tr.is_dumped_item &&
           tr.person_group_id == 0 &&
           tr.last_person_group_id != 0 &&
           tr.person_group_hold_count > 0;
}

// ── 추적기 파라미터 ──────────────────────────────────────────────────
struct TrackerParams {
    // ── 기본 추적 파라미터 ─────────────────────────────────────────────
    // in-class 디폴트 = production_params.h DefaultTrackerParams() 와 동일하게 유지할 것.
    double   stationary_threshold_mm = 30.0;   // 정지 판단 이동거리 (mm)
    uint32_t departure_frame_count   = 20;     // 이탈 판정 연속 정지 프레임 수 (2초 @ 10Hz)
    double   association_max_dist_mm = 400.0;  // 클러스터-트랙 매칭 최대 거리 (mm)
    double   association_width_penalty = 2.0;   // 매칭 시 폭 급변 방지 패널티
    uint32_t lost_age_limit          = 15;     // 매칭 실패 허용 프레임 수 (1.5초)
    uint32_t tentative_confirm_frames = 4;     // 신규 트랙 공개 전 연속 매칭 프레임 수
    uint32_t tentative_max_age        = 8;     // 검증 실패 신규 트랙 최대 생존 프레임 수

    // ── 투기 감지 파라미터 ───────────────────────────────────────────
    bool     enable_dumping_detection      = true;   // 투기 감지 활성화
    double   min_walk_dist_mm              = 500.0;  // 투기 주체 최소 누적 이동거리
    uint32_t min_age_for_dump              = 10;     // 투기 주체 최소 생존 프레임
    uint32_t dump_stationary_frame_count   = 10;     // 투기물 정지 확인 프레임 수 (1초 @ 10Hz)
    double   separation_max_dist_mm        = 600.0;  // 분리 감지 최대 거리 (이전 위치 기준)

    // ── 다리 오인식 방지 파라미터 ────────────────────────────────────
    double   separation_min_dist_from_current_mm = 120.0;  // 투기물이 주체 현재 위치에서 최소한 이 거리 이상 떨어져야 함
    double   min_dump_candidate_width_mm   = 30.0;   // 투기 후보 최소 폭
    uint32_t separation_confirm_frames     = 4;      // 분리 감지 후 이 프레임 동안 독립 존재해야 투기 의심 확정
    double   leg_proximity_radius_mm       = 350.0;  // 확정 단계에서 이 거리 내 사람 트랙 있으면 다리로 간주

    // -- 궤적 이력 + 트랙 복구 파라미터 --
    uint32_t position_history_size         = 30;     // 보존할 최근 위치 수 (궤적 기반 분리 감지)
    double   recovery_max_dist_mm          = 600.0;  // 잠시 lost된 트랙 복구 최대 거리 (mm)
    uint32_t recovery_max_lost_frames      = 10;     // 복구 허용 최대 lost 프레임 수 (정지→재이동 시 ID 보존)

    // -- 폭 감소 감지 (보조 신호) --
    double   width_drop_threshold_mm       = 80.0;   // 1프레임에 이 값 이상 폭 감소 시 감지 (작은 물체 감지)

    // -- 점구름 개수 필터 --
    size_t   min_dump_candidate_points     = 3;      // 투기 후보 최소 점구름 수

    // -- 투기 주체 이탈 확인 --
    double   person_depart_dist_mm         = 450.0;  // 주체가 투기물에서 이 거리 이상 떨어지고 멀어지는 추세면 확정
    double   soft_person_depart_dist_mm    = 350.0;  // 보조 확정 기준 (추세/source lost/PIR 등과 조합)
    double   strong_person_depart_dist_mm  = 600.0;  // 거리만으로 신뢰도가 높은 확정 기준
    uint32_t person_depart_trend_frames    = 4;      // 거리 증가 추세 확인 프레임 수
    double   dumped_item_still_speed_mm    = 15.0;   // 봉투형 물체 정지 판정 속도 허용치(mm/frame)
    uint32_t fast_confirm_min_frames       = 4;      // 직접 증거가 있는 신규 투기물 빠른 확정 프레임
    uint32_t dump_confirm_min_total_matches = 4;     // 확정 전 실제 매칭 관측 최소 수
    double   fast_confirm_depart_mm        = 300.0;  // 빠른 확정 source 이탈 거리
    uint32_t fast_confirm_trend_frames     = 3;      // 빠른 확정 거리 증가 추세 프레임
    double   object_stable_avg_speed_mm    = 25.0;   // 최근 이동량 평균 기반 stable 기준
    uint32_t source_lost_confirm_frames    = 8;      // source lost 후 확정 최소 안정 프레임
    double   near_unrelated_person_hold_mm = 350.0;  // source 아닌 사람 근접 시 확정 hold 반경
    uint32_t dump_candidate_birth_frames   = 30;     // 일반 정지 track이 suspect로 전환 가능한 최대 age

    // -- 속도 벡터 분석 (투사 궤적) --
    double   receding_velocity_threshold   = 20.0;   // 분리 객체가 사람에게서 멀어지는 최소 속도 (mm/frame)

    // -- 칼만 필터 노이즈 파라미터 --
    double   kf_process_noise              = 50.0;   // 프로세스 노이즈 (모델 불확실성)
    double   kf_measure_noise              = 100.0;  // 관측 노이즈 (센서 불확실성)
    double   kf_fallback_dist_mm           = 300.0;  // 칼만 예측 기반 2차 매칭 최대 거리
    double   kf_fallback_max_gate_mm       = 500.0;  // KF fallback 게이트 상한

    // -- Hotspot(투기 지역 우선 감지) --
    double   hotspot_radius_mm             = 500.0;  // 과거 투기 위치 반경 (mm)
    uint32_t hotspot_boost_frames          = 5;      // hotspot 내 의심 객체 확인 프레임 감소량
    uint32_t hotspot_boost_min_frames      = 2;      // hotspot boost 적용 후 최소 확인 프레임 (FP 방지)
    size_t   max_hotspots                  = 20;     // 최대 보존 hotspot 수
    uint32_t hotspot_ttl_frames            = 3000;   // Hotspot 수명 (≈5분 @ 10Hz)

    // ── 알고리즘 상수 (v4→v6: 하드코딩 → 튜닝 가능 파라미터) ──────────
    double   split_radius_margin_mm        = 200.0;  // 클러스터 분열 감지 여유 반경
    double   split_radius_max_mm           = 500.0;  // 클러스터 분열 감지 반경 상한 (FP 방지)
    uint32_t suspect_lost_frames_limit     = 10;     // 의심 트랙이 N프레임 이상 lost → 해제
    uint32_t dumped_item_lost_age_limit    = 200;    // 투기 확정/의심 트랙 전용 수명

    // ── v6 신규: 기존 하드코딩 상수 → 파라미터화 ───────────────────────
    double   active_dump_block_radius_mm   = 300.0;  // 활성 투기물 근처 재감지 차단 반경
    double   person_proximity_hold_radius_mm = 100.0; // 사람 근접 시 트랙 생성 보류 반경
    double   split_match_radius_mm         = 250.0;  // 분열 보조 신호 매칭 반경
    double   new_track_min_width_mm        = 30.0;   // 새 트랙 생성 최소 폭
    double   new_track_max_width_mm        = 1500.0; // 새 트랙 생성 최대 폭
    double   leg_misid_radius_mm           = 450.0;  // 다리 오인 방지 반경
    double   leg_misid_max_width_mm        = 300.0;  // 다리 오인 방지 최대 폭
    double   stationary_track_max_cum_dist_mm = 200.0; // 정지 트랙 궤적 매칭 누적 이동 상한
    double   dump_candidate_max_width_mm   = 900.0;  // 투기 후보 최대 폭

    // 크기 band 별 suspect 확정 요구 프레임.
    //   작을수록 빨리 확정된다(미탐↓ 오탐↑). 현장에서 "쓰레기를 너무 못
    //   잡는다" 면 여기를 1~2 낮추는 것이 가장 직접적이다.
    uint32_t dump_frames_tiny              = 6;
    uint32_t dump_frames_small             = 6;
    uint32_t dump_frames_medium            = 5;
    uint32_t dump_frames_large             = 4;
    uint32_t dump_frames_xl                = 4;

    // ── v13: 15cm 설치 환경 PersonGroup/ObjectCandidate ─────────────
    double   leg_track_max_width_mm        = 300.0;  // 다리 후보 최대 폭
    double   leg_track_min_move_mm         = 20.0;   // 다리 후보 최소 프레임 이동량
    uint32_t leg_track_min_age             = 3;      // 다리 후보 최소 생존 프레임
    double   person_group_pair_radius_mm   = 600.0;  // 다리쌍 그룹화 최대 거리
    double   person_group_new_pair_max_width_mm = 650.0; // 신규 다리쌍 생성 최대 폭
    double   person_group_wide_pair_radius_mm = 1500.0; // 좁은 leg 전용 넓은 보폭 생성 거리
    double   person_group_wide_pair_leg_width_mm = 260.0; // 넓은 보폭 허용 시 각 leg 최대 폭
    double   person_group_wide_pair_max_depth_gap_mm = 500.0; // 넓은 보폭 신규 pair 허용 Y축 최대 차이
    double   person_group_max_width_mm     = 700.0;  // 기존 사람 그룹 표시/유지 최대 폭
    uint32_t person_group_hold_frames      = 18;     // 일시적 보폭/속도 흔들림 시 group 유지 프레임
    uint32_t person_group_wide_pair_birth_gap_frames = 6; // 넓은 보폭 leg 생성 시점 허용 차이
    double   object_candidate_max_width_mm = 250.0;  // 정지 소형 객체 후보 최대 폭
    uint32_t object_candidate_min_stationary_frames = 3; // 정지 객체 후보 판정 프레임

    // ── v16~v18: spike 억제 + direct evidence/보폭 기본값 보정 ──────
    uint32_t min_suspect_callback_frames   = 4;      // visible suspect 최소 관측 프레임
    double   suspect_callback_max_speed_mm = 30.0;   // visible suspect 평균 속도 상한
    double   suspect_callback_max_width_variance = 2500.0; // visible suspect 폭 분산 상한
    uint32_t pending_cluster_confirm_frames = 3;     // 일반 신규 cluster 공개 전 관측 프레임
    uint32_t pending_direct_confirm_frames  = 1;     // direct evidence cluster 공개 전 관측 프레임
    uint32_t pending_static_extra_frames    = 1;     // 거의 정지한 잔상 cluster 추가 보류 프레임
    uint32_t pending_cluster_missed_limit   = 2;     // pending cluster 미관측 허용 프레임
    size_t   pending_cluster_max_count      = 32;    // 고정 상한: heap churn/무한 증가 방지
    double   pending_cluster_match_dist_mm  = 180.0; // pending cluster 연속 관측 매칭 반경
    double   person_group_rejoin_radius_mm  = 1500.0; // 기존 group 재결합/hold 전용 반경
    double   person_group_attach_radius_mm  = 1600.0; // 단일 leg-like track attach 반경
    double   person_group_hold_attach_radius_mm = 1800.0; // 직전 group leg 재부착 전용 반경
    uint32_t trajectory_birth_suspect_min_stationary_frames = 3; // 보조 신호 없는 신규 정지 물체 suspect 최소 정지
    uint32_t trajectory_birth_min_total_matches = 4; // 보조 신호 없는 신규 정지 물체 최소 관측 수
    double   trajectory_birth_suspect_max_width_mm = 650.0; // 보조 신호 없는 trajectory suspect 최대 폭
    double   trajectory_birth_depart_mm = 300.0; // 보조 신호 없는 trajectory suspect source 이탈 거리
};

// ── 사람 엔티티: 15cm LiDAR leg track들을 하나의 사람 상태로 묶는 내부 모델 ──
struct PersonEntity {
    uint32_t id = 0;
    uint32_t primary_track_id = 0;
    uint32_t secondary_track_id = 0;
    double x_mm = 0.0;
    double y_mm = 0.0;
    double vx_mm = 0.0;
    double vy_mm = 0.0;
    double width_mm = 0.0;
    uint32_t age = 0;
    uint32_t lost_count = 0;
    uint32_t hold_count = 0;
    bool confirmed = false;
};

// ── 이탈 이벤트 콜백 타입 ────────────────────────────────────────────
using DepartureCallback = std::function<void(const DepartureEvent&)>;
using DumpingCallback   = std::function<void(const DumpingEvent&)>;
using SuspectCallback   = std::function<void(const DumpingSuspectEvent&)>;

// ── Hotspot / 삭제 트랙 위치 (DumpDetector 와 공유) ─────────────────
struct HotspotEntry {
    double x_mm;
    double y_mm;
    uint32_t frames_remaining;
};

struct DeletedTrackPos {
    double x_mm;
    double y_mm;
    uint32_t frames_remaining;
};

// ── 클러스터 추적기 ──────────────────────────────────────────────────
class ClusterTracker {
public:
    explicit ClusterTracker(const TrackerParams& params = TrackerParams{});

    /**
     * @brief 새 프레임의 클러스터를 기존 트랙에 매칭하고 상태를 갱신한다.
     * @param clusters       현재 프레임의 DBSCAN 클러스터 결과
     * @param[out] dep_events   이번 프레임에서 발생한 이탈 이벤트
     * @param[out] dump_events  이번 프레임에서 확정된 투기 이벤트
     */
    void Update(const std::vector<Cluster>& clusters,
                std::vector<DepartureEvent>& dep_events,
                std::vector<DumpingEvent>& dump_events);

    void SetDepartureCallback(DepartureCallback cb);
    void SetDumpingCallback(DumpingCallback cb);
    void SetSuspectCallback(SuspectCallback cb);

    const std::vector<Track>& GetTracks() const { return tracks_; }
    uint32_t GetFrameCount() const { return frame_count_; }

    /**
     * @brief 투기 관련 플래그를 되돌려 일반 트랙 수명으로 복귀시킨다.
     * @return 해당 ID 트랙을 찾아 되돌렸으면 true.
     *
     *   v55 재검증이 오탐으로 **취소**한 트랙에 쓴다. 취소했는데 플래그를
     *   그대로 두면:
     *     - `json_packet` 이 `is_dumped_item` 트랙을 `confirmed`/`lost_count`
     *       게이트에서 면제하므로 센서가 놓쳐도 계속 `type:"dumped"` 로 송신
     *     - 수명이 `dumped_item_lost_age_limit`(기본 200프레임 = 20초)로 늘어남
     *   → 서버 전송만 막히고 Unity 화면에는 오탐이 20초간 남는다.
     *
     *   `source_id` 까지 되돌려야 `IsBackgroundResidualTrack()` 등의
     *   판정이 정상 트랙과 동일하게 동작한다 (§ 176-204 헬퍼 참조).
     */
    bool ClearDumpFlags(uint32_t track_id) {
        for (auto& tr : tracks_) {
            if (tr.id != track_id) continue;
            tr.is_dumped_item  = false;
            tr.is_dump_suspect = false;
            tr.source_id       = -1;
            return true;
        }
        return false;
    }

    void SetParams(const TrackerParams& p) { params_ = p; }

private:
    TrackerParams      params_;
    std::vector<Track> tracks_;
    std::vector<PersonEntity> person_entities_;
    uint32_t           next_id_     = 1;  // 0 예약 (untracked)
    uint32_t           wrap_count_  = 0;  // next_id_ 가 UINT32_MAX 를 넘은 횟수 (운영 감시용)
    uint32_t           frame_count_ = 0;
    DepartureCallback  dep_callback_;
    DumpingCallback    dump_callback_;
    SuspectCallback    suspect_callback_;

    // -- Hotspot (과거 투기 위치, TTL 기반) --
    std::vector<HotspotEntry> hotspot_positions_;

    bool IsInHotspot(double x, double y) const;

    // 삭제된 일반 트랙의 최근 위치 버퍼 (오인식 방지용)
    std::vector<DeletedTrackPos> deleted_positions_;

    // ── 영속 스크래치 버퍼 (프레임당 heap alloc 제거) ────────────────
    //   매 프레임 Update()가 초기화(assign/clear)한 후 재사용한다.
    //   의미는 로컬 변수와 동일하지만, capacity 를 유지해 최악 1프레임당
    //   수 KB 할당/해제가 발생하던 것을 상수 시간 재사용으로 바꾼다.
    struct AssociatePair {
        double dist_sq;
        size_t ci;
        size_t ti;
    };
    std::vector<AssociatePair> scratch_pairs_;
    std::vector<int>           scratch_cluster_to_track_;  // [ci] = ti 또는 -1
    std::vector<bool>          scratch_track_matched_;     // [ti] = true/false
    std::vector<bool>          scratch_cluster_claimed_;   // [ci] = true/false (분리 감지 단계)

    struct PendingCluster {
        double x_mm = 0.0;
        double y_mm = 0.0;
        double first_x_mm = 0.0;
        double first_y_mm = 0.0;
        double width_mm = 0.0;
        uint8_t points = 0;
        uint8_t seen_count = 0;
        uint8_t missed_count = 0;
        uint32_t first_frame = 0;
        bool direct_evidence = false;
    };
    std::vector<PendingCluster> pending_clusters_;

    static uint64_t NowMs();

    uint32_t AllocId() {
        uint32_t id = next_id_++;
        if (next_id_ == 0) {
            // UINT32_MAX 오버플로 → 1로 리셋 (0 예약). 10Hz 기준 수 년 단위로 발생.
            // 진짜 충돌(살아있는 트랙 ID 재사용) 감지는 별도 과제지만, 최소한 운영 로그
            // 에서 wrap 자체가 관측 가능하도록 경고를 남긴다.
            next_id_ = 1;
            wrap_count_++;
            std::fprintf(stderr,
                "[TRACKER] ⚠ Track ID counter wrapped (wrap #%u). "
                "장기 운영 중. 현재 tracks=%zu — 충돌 가능성 감시 필요.\n",
                wrap_count_, tracks_.size());
        }
        return id;
    }

    // 매칭 헬퍼 — scratch_* 멤버를 공유 상태로 사용한다.
    void AssociateGreedy(const std::vector<Cluster>& clusters);
    void UpdatePersonGroups();
    bool IsObjectCandidate(const Track& tr) const;
    bool IsLegCandidate(const Track& tr) const;
    bool IsLegLikeForPerson(const Track& tr) const;
    bool IsPersonPairCandidate(const Track& tr) const;
    bool HasDirectEvidenceForCluster(const Cluster& cluster) const;
    bool ShouldHoldPendingCluster(const Cluster& cluster, bool direct_evidence);
    void PrunePendingClusters();

    // 투기 감지는 DumpDetector 에 위임 (dump_detector.h)
};

} // namespace ecowarden
