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
 * @file  production_params.h
 * @brief 운영 파라미터 단일 출처 (Single Source of Truth)
 */

#pragma once

#include "scan_processor.h"
#include "background_filter.h"
#include "camera_module.h"
#include "cluster_tracker.h"
#include "servo_controller.h"
#include "zone_policy.h"

#include <cstdlib>
#include <string>

namespace ecowarden {

inline double EnvDouble(const char* name, double fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') return fallback;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return (end != value) ? parsed : fallback;
}

inline uint32_t EnvU32(const char* name, uint32_t fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') return fallback;
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(value, &end, 10);
    return (end != value) ? static_cast<uint32_t>(parsed) : fallback;
}

inline bool EnvBool(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') return fallback;
    return value[0] == '1' || value[0] == 't' || value[0] == 'T' ||
           value[0] == 'y' || value[0] == 'Y';
}

inline std::string EnvString(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') return fallback;
    return value;
}

// ── ScanProcessor: 노이즈 필터 ───────────────────────────────────────
inline FilterParams DefaultFilterParams() {
    FilterParams fp;
    fp.min_distance_mm      = 150;
    fp.max_distance_mm      = 12000;
    fp.min_intensity        = 10;
    fp.fov_min_deg          = 0.0f;
    fp.fov_max_deg          = 180.0f;
    fp.min_cluster_width_mm = 50.0;
    fp.max_cluster_width_mm = 1200.0;
    fp.merge_radius_mm      = EnvDouble("ECOWARDEN_SCAN_MERGE_RADIUS_MM", 120.0);
    fp.leg_pair_merge_radius_mm = 0.0;
    fp.leg_pair_max_width_mm    = 300.0;
    fp.leg_pair_max_points      = 30;
    fp.static_bg_margin_mm     = EnvDouble("ECOWARDEN_STATIC_BG_MARGIN_MM", 150.0);
    fp.static_bg_ema_alpha     = 0.02f;
    fp.static_bg_min_confidence = static_cast<uint8_t>(
        EnvU32("ECOWARDEN_STATIC_BG_MIN_CONFIDENCE", 10));
    fp.static_bg_edge_guard_slots = static_cast<uint8_t>(
        EnvU32("ECOWARDEN_STATIC_BG_EDGE_GUARD_SLOTS", 1));
    fp.static_bg_jump_reject_mm = EnvU32("ECOWARDEN_STATIC_BG_JUMP_REJECT_MM", 500);
    fp.reverse_angle = EnvBool("ECOWARDEN_REVERSE_LIDAR_ANGLE", true);
    return fp;
}

// ── ScanProcessor: DBSCAN ────────────────────────────────────────────
inline DBSCANParams DefaultDBSCANParams() {
    DBSCANParams dp;
    dp.epsilon_mm = 100.0;
    dp.min_points = 3;
    return dp;
}

// ── BackgroundFilter ─────────────────────────────────────────────────
inline BackgroundFilterParams DefaultBackgroundFilterParams() {
    BackgroundFilterParams bgp;
    bgp.learning_frames        = 50;     // 배경 학습 시간
    bgp.match_radius_mm        = 80.0;
    bgp.enable_adaptive        = true;
    bgp.add_after_frames       = 300;
    bgp.remove_after_absent    = 500;
    bgp.enable_dual_background = true;
    bgp.long_term_add_frames   = 1000;
    bgp.long_term_remove_after = 3000;
    bgp.residual_filter_after_frames = EnvU32(
        "ECOWARDEN_BG_RESIDUAL_FILTER_AFTER_FRAMES", 40);
    bgp.residual_filter_radius_mm = EnvDouble(
        "ECOWARDEN_BG_RESIDUAL_FILTER_RADIUS_MM", 140.0);
    bgp.residual_filter_max_width_mm = EnvDouble(
        "ECOWARDEN_BG_RESIDUAL_FILTER_MAX_WIDTH_MM", 350.0);
    return bgp;
}

// ── ClusterTracker ───────────────────────────────────────────────────
inline TrackerParams DefaultTrackerParams() {
    TrackerParams tp;
    tp.person_group_hold_frames = EnvU32(
        "ECOWARDEN_PERSON_GROUP_HOLD_FRAMES",
        tp.person_group_hold_frames);
    tp.person_group_wide_pair_radius_mm = EnvDouble(
        "ECOWARDEN_PERSON_GROUP_WIDE_PAIR_RADIUS_MM",
        tp.person_group_wide_pair_radius_mm);
    tp.person_group_wide_pair_max_depth_gap_mm = EnvDouble(
        "ECOWARDEN_PERSON_GROUP_WIDE_PAIR_MAX_DEPTH_GAP_MM",
        tp.person_group_wide_pair_max_depth_gap_mm);
    tp.person_group_wide_pair_birth_gap_frames = EnvU32(
        "ECOWARDEN_PERSON_GROUP_WIDE_PAIR_BIRTH_GAP_FRAMES",
        tp.person_group_wide_pair_birth_gap_frames);
    tp.person_group_rejoin_radius_mm = EnvDouble(
        "ECOWARDEN_PERSON_GROUP_REJOIN_RADIUS_MM",
        tp.person_group_rejoin_radius_mm);
    tp.person_group_attach_radius_mm = EnvDouble(
        "ECOWARDEN_PERSON_GROUP_ATTACH_RADIUS_MM",
        tp.person_group_attach_radius_mm);
    tp.person_group_hold_attach_radius_mm = EnvDouble(
        "ECOWARDEN_PERSON_GROUP_HOLD_ATTACH_RADIUS_MM",
        tp.person_group_hold_attach_radius_mm);
    tp.association_width_penalty = EnvDouble(
        "ECOWARDEN_ASSOCIATION_WIDTH_PENALTY",
        tp.association_width_penalty);

    tp.fast_confirm_min_frames = EnvU32(
        "ECOWARDEN_FAST_CONFIRM_MIN_FRAMES",
        tp.fast_confirm_min_frames);
    tp.fast_confirm_depart_mm = EnvDouble(
        "ECOWARDEN_FAST_CONFIRM_DEPART_MM",
        tp.fast_confirm_depart_mm);
    tp.fast_confirm_trend_frames = EnvU32(
        "ECOWARDEN_FAST_CONFIRM_TREND_FRAMES",
        tp.fast_confirm_trend_frames);
    tp.dump_confirm_min_total_matches = EnvU32(
        "ECOWARDEN_DUMP_CONFIRM_MIN_TOTAL_MATCHES",
        tp.dump_confirm_min_total_matches);
    tp.source_lost_confirm_frames = EnvU32(
        "ECOWARDEN_SOURCE_LOST_CONFIRM_FRAMES",
        tp.source_lost_confirm_frames);
    tp.trajectory_birth_depart_mm = EnvDouble(
        "ECOWARDEN_TRAJECTORY_BIRTH_DEPART_MM",
        tp.trajectory_birth_depart_mm);
    tp.trajectory_birth_suspect_max_width_mm = EnvDouble(
        "ECOWARDEN_TRAJECTORY_BIRTH_SUSPECT_MAX_WIDTH_MM",
        tp.trajectory_birth_suspect_max_width_mm);
    tp.trajectory_birth_min_total_matches = EnvU32(
        "ECOWARDEN_TRAJECTORY_BIRTH_MIN_TOTAL_MATCHES",
        tp.trajectory_birth_min_total_matches);
    tp.pending_cluster_confirm_frames = EnvU32(
        "ECOWARDEN_PENDING_CLUSTER_CONFIRM_FRAMES",
        tp.pending_cluster_confirm_frames);
    tp.pending_static_extra_frames = EnvU32(
        "ECOWARDEN_PENDING_STATIC_EXTRA_FRAMES",
        tp.pending_static_extra_frames);
    // 시연/저노이즈 환경에서 신규 트랙 공개 지연을 줄일 때 사용 (기본 4프레임).
    tp.tentative_confirm_frames = EnvU32(
        "ECOWARDEN_TENTATIVE_CONFIRM_FRAMES",
        tp.tentative_confirm_frames);
    return tp;
}

// ── ServoController ─────────────────────────────────────────────────
inline ServoParams DefaultServoParams() {
    ServoParams sp;
    sp.enable = EnvBool("ECOWARDEN_SERVO_ENABLE", sp.enable);
    sp.mirror = EnvBool("ECOWARDEN_SERVO_MIRROR", sp.mirror);
    sp.backend = EnvString("ECOWARDEN_SERVO_BACKEND", sp.backend);
    sp.serial_device = EnvString("ECOWARDEN_SERVO_SERIAL_DEVICE", sp.serial_device);
    sp.serial_baud = EnvU32("ECOWARDEN_SERVO_SERIAL_BAUD", sp.serial_baud);
    sp.serial_boot_ms = EnvU32("ECOWARDEN_SERVO_SERIAL_BOOT_MS", sp.serial_boot_ms);
    sp.pwm_chip = EnvString("ECOWARDEN_SERVO_PWM_CHIP", sp.pwm_chip);
    sp.pwm_channel = EnvU32("ECOWARDEN_SERVO_PWM_CHANNEL", sp.pwm_channel);
    sp.period_ns = EnvU32("ECOWARDEN_SERVO_PERIOD_NS", sp.period_ns);
    sp.min_pulse_us = EnvU32("ECOWARDEN_SERVO_MIN_PULSE_US", sp.min_pulse_us);
    sp.max_pulse_us = EnvU32("ECOWARDEN_SERVO_MAX_PULSE_US", sp.max_pulse_us);
    if (sp.min_pulse_us >= sp.max_pulse_us) {
        sp.min_pulse_us = 544;
        sp.max_pulse_us = 2400;
    }
    sp.zone_hysteresis_deg = EnvDouble("ECOWARDEN_SERVO_ZONE_HYSTERESIS_DEG",
                                       sp.zone_hysteresis_deg);
    sp.slew_step_deg = EnvDouble("ECOWARDEN_SERVO_SLEW_STEP_DEG", sp.slew_step_deg);
    sp.slew_step_ms = EnvU32("ECOWARDEN_SERVO_SLEW_STEP_MS", sp.slew_step_ms);
    sp.ms_per_deg = EnvDouble("ECOWARDEN_SERVO_MS_PER_DEG", sp.ms_per_deg);
    sp.settle_ms = EnvU32("ECOWARDEN_SERVO_SETTLE_MS", sp.settle_ms);
    sp.max_move_ms = EnvU32("ECOWARDEN_SERVO_MAX_MOVE_MS", sp.max_move_ms);
    sp.suspect_dir = EnvString("ECOWARDEN_SERVO_SUSPECT_DIR", sp.suspect_dir);
    sp.dump_dir = EnvString("ECOWARDEN_SERVO_DUMP_DIR", sp.dump_dir);
    sp.person_dir = EnvString("ECOWARDEN_SERVO_PERSON_DIR", sp.person_dir);
    sp.person_capture_interval_ms = EnvU32(
        "ECOWARDEN_SERVO_PERSON_CAPTURE_INTERVAL_MS",
        sp.person_capture_interval_ms);
    sp.person_session_timeout_ms = EnvU32(
        "ECOWARDEN_SERVO_PERSON_SESSION_TIMEOUT_MS",
        sp.person_session_timeout_ms);
    return sp;
}

// ── CameraModule: 블랙박스(전후 N초) 링버퍼 ─────────────────────────
inline BlackboxParams DefaultBlackboxParams() {
    BlackboxParams bp;
    bp.enable = EnvBool("ECOWARDEN_BLACKBOX_ENABLE", bp.enable);
    const uint32_t sec = EnvU32("ECOWARDEN_BLACKBOX_SEC", 10);
    bp.pre_seconds  = EnvU32("ECOWARDEN_BLACKBOX_PRE_SEC", sec);
    bp.post_seconds = EnvU32("ECOWARDEN_BLACKBOX_POST_SEC", sec);
    bp.fps = EnvDouble("ECOWARDEN_BLACKBOX_FPS", bp.fps);
    bp.jpeg_quality = static_cast<int>(
        EnvU32("ECOWARDEN_BLACKBOX_JPEG_QUALITY",
               static_cast<uint32_t>(bp.jpeg_quality)));
    return bp;
}

// ── ZonePolicy: 금지구역(존) 침입 탐지 ──────────────────────────────
inline ZonePolicyParams DefaultZonePolicyParams() {
    ZonePolicyParams zp;
    zp.restricted_zones = EnvString("ECOWARDEN_RESTRICTED_ZONES",
                                    zp.restricted_zones);
    zp.intrusion_min_frames = EnvU32("ECOWARDEN_INTRUSION_MIN_FRAMES",
                                     zp.intrusion_min_frames);
    zp.intrusion_repeat_ms = EnvU32("ECOWARDEN_INTRUSION_REPEAT_MS",
                                    zp.intrusion_repeat_ms);
    zp.forget_after_ms = EnvU32("ECOWARDEN_INTRUSION_FORGET_MS",
                                zp.forget_after_ms);
    return zp;
}

} // namespace ecowarden
