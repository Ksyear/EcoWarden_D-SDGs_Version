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
 * @file  production_params.h
 * @brief 운영 파라미터 단일 출처 (Single Source of Truth)
 */

#pragma once

#include "scan_processor.h"
#include "background_filter.h"
#include "camera_module.h"
#include "cluster_tracker.h"
#include "dump_validation.h"
#include "evidence_vault.h"
#include "face_masking.h"
#include "object_classifier.h"
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

    // ── 투기 감지 감도 프리셋 ────────────────────────────────────────
    //
    //   ECOWARDEN_DUMP_SENSITIVITY = high | normal | low   (기본 high)
    //
    //   여러 게이트를 따로 만지면 서로 상쇄되기 쉬워서, 함께 움직여야
    //   하는 값들을 한 덩어리로 묶었다.
    //
    //   기본을 high 로 둔 것은 의도적이다. 이 시스템은 **1차 스크리너**이고
    //   최종 판단은 관리자가 증거 사진으로 한다. 놓친 투기는 되돌릴 수
    //   없지만, 오탐은 사람이 사진 한 번 보고 거르면 된다.
    //   → 줄여야 할 것은 오탐이 아니라 미탐.
    //
    //   오탐이 감당 안 되면 normal, 시연에서 확실한 것만 잡으려면 low.
    //   개별 값은 아래 env 로 프리셋을 덮어쓸 수 있다.
    {
        const std::string sens =
            EnvString("ECOWARDEN_DUMP_SENSITIVITY", "high");
        if (sens == "high") {
            tp.dump_frames_tiny   = 3;
            tp.dump_frames_small  = 3;
            tp.dump_frames_medium = 3;
            tp.dump_frames_large  = 2;
            tp.dump_frames_xl     = 2;
            // 2 로 내리면 반사 스파이크가 두 번만 찍혀도 확정된다
            //   (회귀 K_reflective_spike_near_path 로 확인). 3 이 하한선.
            tp.dump_confirm_min_total_matches = 3;
            tp.fast_confirm_min_frames        = 2;
            tp.source_lost_confirm_frames     = 4;
            // trajectory_birth_suspect_min_stationary_frames 는 낮추지 않는다.
            //   "보조 신호 없이 갑자기 나타난 정지 물체" 를 suspect 로 올릴지
            //   결정하는 게이트인데, 반사 스파이크가 정확히 그 모양이다.
            //   3→2 로 낮췄더니 회귀 K_reflective_spike_near_path 가 투기로
            //   확정됐다. 사람이 내려놓은 쓰레기는 분리 신호(폭 감소/분열/
            //   궤적 근접)가 함께 오므로 이 게이트를 안 거쳐도 잡힌다.
        } else if (sens == "low") {
            tp.dump_frames_tiny   = 8;
            tp.dump_frames_small  = 8;
            tp.dump_frames_medium = 6;
            tp.dump_frames_large  = 5;
            tp.dump_frames_xl     = 5;
            tp.dump_confirm_min_total_matches = 6;
            tp.source_lost_confirm_frames     = 10;
        }
        // normal 은 구조체 기본값 그대로
    }

    // 개별 오버라이드 (프리셋보다 우선)
    tp.min_dump_candidate_width_mm = EnvDouble(
        "ECOWARDEN_DUMP_MIN_WIDTH_MM", tp.min_dump_candidate_width_mm);
    tp.dump_candidate_max_width_mm = EnvDouble(
        "ECOWARDEN_DUMP_MAX_WIDTH_MM", tp.dump_candidate_max_width_mm);
    tp.dump_frames_tiny   = EnvU32("ECOWARDEN_DUMP_FRAMES_TINY",
                                   tp.dump_frames_tiny);
    tp.dump_frames_small  = EnvU32("ECOWARDEN_DUMP_FRAMES_SMALL",
                                   tp.dump_frames_small);
    tp.dump_frames_medium = EnvU32("ECOWARDEN_DUMP_FRAMES_MEDIUM",
                                   tp.dump_frames_medium);
    tp.dump_frames_large  = EnvU32("ECOWARDEN_DUMP_FRAMES_LARGE",
                                   tp.dump_frames_large);
    tp.dump_frames_xl     = EnvU32("ECOWARDEN_DUMP_FRAMES_XL",
                                   tp.dump_frames_xl);
    // 반사 오탐과 직결되는 값 — 내리면 오탐이 확 는다. 기본 유지 권장.
    tp.trajectory_birth_suspect_min_stationary_frames = EnvU32(
        "ECOWARDEN_TRAJECTORY_BIRTH_SUSPECT_MIN_STATIONARY_FRAMES",
        tp.trajectory_birth_suspect_min_stationary_frames);
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
    bp.max_concurrent_saves = EnvU32("ECOWARDEN_BLACKBOX_MAX_CONCURRENT",
                                     bp.max_concurrent_saves);
    return bp;
}

// ── CameraModule: 사람 머리 위 ID 라벨 ──────────────────────────────
inline HeadLabelParams DefaultHeadLabelParams() {
    HeadLabelParams hp;
    hp.enable        = EnvBool("ECOWARDEN_HEAD_LABEL_ENABLE", hp.enable);
    hp.hfov_deg      = EnvDouble("ECOWARDEN_CAMERA_HFOV_DEG", hp.hfov_deg);
    hp.vfov_deg      = EnvDouble("ECOWARDEN_CAMERA_VFOV_DEG", hp.vfov_deg);
    hp.cam_height_m  = EnvDouble("ECOWARDEN_CAMERA_HEIGHT_M", hp.cam_height_m);
    hp.head_height_m = EnvDouble("ECOWARDEN_HEAD_HEIGHT_M", hp.head_height_m);
    hp.pitch_deg     = EnvDouble("ECOWARDEN_CAMERA_PITCH_DEG", hp.pitch_deg);
    hp.image_mirror  = EnvBool("ECOWARDEN_HEAD_LABEL_IMG_MIRROR",
                               hp.image_mirror);
    return hp;
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

// ── DumpValidator: 투기 확정 후 재검증 (오탐 차단) ──────────────────
inline DumpValidationParams DefaultDumpValidationParams() {
    DumpValidationParams dv;

    // 감지 감도와 함께 움직인다. high 에서 재검증이 빡빡하면 애써 확정한
    // 투기를 다시 취소해 버려서 감도를 올린 의미가 없어진다.
    //   - 관찰 창을 30→20프레임(3초→2초)으로 줄여 확정이 빨리 나가고
    //   - 재관측 비율 하한을 0.5→0.3 으로 낮추고 (LiDAR 가 몇 프레임 놓쳐도 통과)
    //   - 허용 이동량을 300→500mm 로 넓힌다 (centroid 흔들림 흡수)
    //     실제로 현장 로그에서 object 15 가 "300mm 초과 이동" 으로 취소됐다.
    {
        const std::string sens =
            EnvString("ECOWARDEN_DUMP_SENSITIVITY", "high");
        if (sens == "high") {
            // 관찰 창은 줄이고(확정이 빨리 나감) 이동 허용은 넓히되,
            // **재관측 비율은 낮추지 않는다.**
            //   이 비율이 반사·고스트를 걸러내는 유일한 장치다. 감지 게이트와
            //   함께 풀어 버리면 안전망이 사라진다 — 실제로 0.3 으로 낮췄더니
            //   회귀 K_reflective_spike_near_path(반사 스파이크)가 투기로
            //   확정됐다. 젖은 바닥·유리·금속이 많은 현장에서는 이게 계속 뜬다.
            dv.validate_frames    = 20;
            dv.min_present_ratio  = 0.5;
            dv.max_move_mm        = 500.0;
        } else if (sens == "low") {
            dv.validate_frames    = 40;
            dv.min_present_ratio  = 0.6;
            dv.max_move_mm        = 250.0;
        }
    }

    dv.enable = EnvBool("ECOWARDEN_DUMP_VALIDATE", dv.enable);
    dv.validate_frames = EnvU32("ECOWARDEN_DUMP_VALIDATE_FRAMES",
                                dv.validate_frames);
    dv.max_move_mm = EnvDouble("ECOWARDEN_DUMP_VALIDATE_MAX_MOVE_MM",
                               dv.max_move_mm);
    dv.min_present_ratio = EnvDouble("ECOWARDEN_DUMP_VALIDATE_MIN_PRESENT",
                                     dv.min_present_ratio);
    dv.high_present_ratio = EnvDouble("ECOWARDEN_DUMP_VALIDATE_HIGH_PRESENT",
                                      dv.high_present_ratio);
    dv.source_departed_mm = EnvDouble("ECOWARDEN_DUMP_VALIDATE_DEPART_MM",
                                      dv.source_departed_mm);
    return dv;
}

// ── FaceMasker: 증거 사진 얼굴 마스킹 (프라이버시 2계층) ────────────
//
//   기본값이 "켜짐 + fail-closed" 인 것은 의도적이다. 마스킹을 켜 놓고
//   검출기가 없어서 원본이 그대로 나가는 상황이 가장 나쁘기 때문에,
//   그 경우 상단 영역을 통째로 가린다(ECOWARDEN_FACE_MASK_FALLBACK=0
//   으로 끄면 아예 저장하지 않는다).
//
inline FaceMaskParams DefaultFaceMaskParams() {
    FaceMaskParams fm;
    fm.enable = EnvBool("ECOWARDEN_FACE_MASK", fm.enable);
    fm.mode = ParseFaceMaskMode(
        EnvString("ECOWARDEN_FACE_MASK_MODE",
                  FaceMaskModeToString(fm.mode)));
    fm.cascade_path = EnvString("ECOWARDEN_FACE_MASK_CASCADE",
                                fm.cascade_path);
    fm.dnn_config  = EnvString("ECOWARDEN_FACE_MASK_DNN_CONFIG",
                               fm.dnn_config);
    fm.dnn_weights = EnvString("ECOWARDEN_FACE_MASK_DNN_WEIGHTS",
                               fm.dnn_weights);
    fm.dnn_confidence = EnvDouble("ECOWARDEN_FACE_MASK_DNN_CONF",
                                  fm.dnn_confidence);
    fm.scale_factor  = EnvDouble("ECOWARDEN_FACE_MASK_SCALE",
                                 fm.scale_factor);
    fm.min_neighbors = static_cast<int>(
        EnvU32("ECOWARDEN_FACE_MASK_NEIGHBORS",
               static_cast<uint32_t>(fm.min_neighbors)));
    fm.min_face_px = static_cast<int>(
        EnvU32("ECOWARDEN_FACE_MASK_MIN_PX",
               static_cast<uint32_t>(fm.min_face_px)));
    fm.expand_ratio  = EnvDouble("ECOWARDEN_FACE_MASK_EXPAND",
                                 fm.expand_ratio);
    fm.detect_max_width = static_cast<int>(
        EnvU32("ECOWARDEN_FACE_MASK_DETECT_WIDTH",
               static_cast<uint32_t>(fm.detect_max_width)));
    fm.label_faces  = EnvBool("ECOWARDEN_FACE_MASK_LABEL", fm.label_faces);
    fm.label_prefix = EnvString("ECOWARDEN_FACE_MASK_LABEL_PREFIX",
                                fm.label_prefix);
    fm.label_scale  = EnvDouble("ECOWARDEN_FACE_MASK_LABEL_SCALE",
                                fm.label_scale);
    fm.blur_strength = EnvDouble("ECOWARDEN_FACE_MASK_STRENGTH",
                                 fm.blur_strength);
    fm.pixelate_blocks = static_cast<int>(
        EnvU32("ECOWARDEN_FACE_MASK_BLOCKS",
               static_cast<uint32_t>(fm.pixelate_blocks)));
    fm.require_detector = EnvBool("ECOWARDEN_FACE_MASK_REQUIRE",
                                  fm.require_detector);
    fm.fallback_mask_upper = EnvBool("ECOWARDEN_FACE_MASK_FALLBACK",
                                     fm.fallback_mask_upper);
    fm.fallback_upper_ratio = EnvDouble("ECOWARDEN_FACE_MASK_FALLBACK_RATIO",
                                        fm.fallback_upper_ratio);
    return fm;
}

// ── ObjectClassifier: 사람/동물/불명 분류 ───────────────────────────
inline ClassifierParams DefaultClassifierParams() {
    ClassifierParams cp;
    cp.enable = EnvBool("ECOWARDEN_CLASSIFY", cp.enable);
    cp.dnn_model  = EnvString("ECOWARDEN_CLASSIFY_MODEL",  cp.dnn_model);
    cp.dnn_config = EnvString("ECOWARDEN_CLASSIFY_CONFIG", cp.dnn_config);
    cp.dnn_labels = EnvString("ECOWARDEN_CLASSIFY_LABELS", cp.dnn_labels);
    cp.dnn_confidence = EnvDouble("ECOWARDEN_CLASSIFY_CONF",
                                  cp.dnn_confidence);
    cp.dnn_input_size = static_cast<int>(
        EnvU32("ECOWARDEN_CLASSIFY_INPUT",
               static_cast<uint32_t>(cp.dnn_input_size)));

    cp.person_min_width_mm = EnvDouble("ECOWARDEN_CLASSIFY_PERSON_MIN_W",
                                       cp.person_min_width_mm);
    cp.person_max_width_mm = EnvDouble("ECOWARDEN_CLASSIFY_PERSON_MAX_W",
                                       cp.person_max_width_mm);
    cp.person_min_speed_mm_s = EnvDouble("ECOWARDEN_CLASSIFY_PERSON_MIN_V",
                                         cp.person_min_speed_mm_s);
    cp.person_max_speed_mm_s = EnvDouble("ECOWARDEN_CLASSIFY_PERSON_MAX_V",
                                         cp.person_max_speed_mm_s);
    cp.animal_max_width_mm = EnvDouble("ECOWARDEN_CLASSIFY_ANIMAL_MAX_W",
                                       cp.animal_max_width_mm);
    cp.animal_min_speed_mm_s = EnvDouble("ECOWARDEN_CLASSIFY_ANIMAL_MIN_V",
                                         cp.animal_min_speed_mm_s);
    cp.noise_max_age_frames = EnvU32("ECOWARDEN_CLASSIFY_NOISE_AGE",
                                     cp.noise_max_age_frames);
    cp.person_min_match_frames = EnvU32("ECOWARDEN_CLASSIFY_PERSON_FRAMES",
                                        cp.person_min_match_frames);
    cp.static_min_stationary_frames =
        EnvU32("ECOWARDEN_CLASSIFY_STATIC_FRAMES",
               cp.static_min_stationary_frames);
    cp.lidar_confidence_cap = EnvDouble("ECOWARDEN_CLASSIFY_LIDAR_CAP",
                                        cp.lidar_confidence_cap);
    return cp;
}

// ── EvidenceVault: 원본 암호화 보관 (프라이버시 3계층) ──────────────
//
//   기본값이 꺼짐인 것은 의도적이다. 키 설정 없이 켜 두면 fail-closed 로
//   어차피 저장이 안 되고, "원본을 아예 안 남긴다"가 가장 보수적인
//   기본 동작이기 때문이다. 운영자가 명시적으로 켜고 키를 넣어야 한다.
//
inline EvidenceVaultParams DefaultEvidenceVaultParams() {
    EvidenceVaultParams vp;
    vp.enable      = EnvBool("ECOWARDEN_EVIDENCE_VAULT", vp.enable);
    vp.vault_dir   = EnvString("ECOWARDEN_EVIDENCE_DIR", vp.vault_dir);
    vp.access_log  = EnvString("ECOWARDEN_EVIDENCE_LOG", vp.access_log);
    vp.retain_days = EnvU32("ECOWARDEN_EVIDENCE_RETAIN_DAYS",
                            vp.retain_days);
    vp.key_hex     = EnvString("ECOWARDEN_EVIDENCE_KEY", vp.key_hex);
    vp.key_file    = EnvString("ECOWARDEN_EVIDENCE_KEY_FILE", vp.key_file);
    return vp;
}

} // namespace ecowarden
