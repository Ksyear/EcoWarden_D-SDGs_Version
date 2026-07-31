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
 * @file  background_filter.h
 * @brief 배경 학습 기반 고정 물체 필터링
 * @date  2026
 *
 * 동작:
 *   1. 학습 단계 (learning_frames 프레임 동안):
 *      매 프레임의 클러스터를 누적하여 배경 맵 구축
 *   2. 운용 단계:
 *      배경 맵에 등록된 위치와 가까운 클러스터를 제거
 */

#pragma once

#include "scan_processor.h"

#include <vector>
#include <utility>
#include <cstdint>

namespace ecowarden {

// ── 배경 필터 파라미터 ──────────────────────────────────────────────
struct BackgroundFilterParams {
    // in-class 디폴트 = production_params.h DefaultBackgroundFilterParams() 와 동일하게 유지할 것.
    uint32_t learning_frames = 50;      // 배경 학습 프레임 수
    double   match_radius_mm = 80.0;    // 배경 매칭 반경 (mm)

    // ── 적응형 배경 갱신 (v6 재활성화) ─────────────────────────────
    bool     enable_adaptive  = true;   // 운용 중 배경 자동 갱신 활성화
    uint32_t add_after_frames = 300;    // 30초 관측 시 배경 등록 (보수적)
    uint32_t remove_after_absent = 500; // 50초 미관측 시 배경 제거

    // ── 이중 배경 모델 (장기) ────────────────────────────────────────
    bool     enable_dual_background     = true;    // 이중 배경 모델 활성화
    uint32_t long_term_add_frames       = 1000;    // 100초 관측 시 장기 배경 등록
    uint32_t long_term_remove_after     = 3000;    // 5분 미관측 시 장기 배경 제거

    // ── 배경 잔상 억제 ───────────────────────────────────────────────
    uint32_t residual_filter_after_frames = 40;     // 4초 이상 같은 위치면 잔상 후보로 제거
    double   residual_filter_radius_mm    = 140.0;  // 후보 매칭 반경
    double   residual_filter_max_width_mm = 350.0;  // 중간 크기 배경 잔상 상한
};

// ── 배경 맵 엔트리 ──────────────────────────────────────────────────
struct BackgroundEntry {
    double   x_mm;
    double   y_mm;
    double   width_mm;
    uint32_t seen_count  = 0;   // 연속 관측 프레임 수 (적응형 등록용)
    uint32_t absent_count = 0;  // 연속 미관측 프레임 수 (적응형 제거용)
    bool     confirmed   = false; // 학습 단계에서 등록된 확정 배경인지
};

// ── 배경 필터 ───────────────────────────────────────────────────────
class BackgroundFilter {
public:
    explicit BackgroundFilter(const BackgroundFilterParams& params = BackgroundFilterParams{});

    /**
     * @brief 클러스터를 배경 맵에 학습시키거나, 배경을 필터링한다.
     *
     * @param clusters  입력 클러스터 (운용 단계에서 배경 클러스터 제거됨)
     * @return true: 학습 완료 후 운용 단계, false: 아직 학습 중 (클러스터 무시)
     */
    bool Apply(std::vector<Cluster>& clusters);

    bool IsLearning() const { return frame_count_ < params_.learning_frames; }
    bool IsReady()    const { return !IsLearning(); }

    uint32_t GetFrameCount()      const { return frame_count_; }
    size_t   GetBackgroundCount() const { return background_map_.size(); }

    void SetParams(const BackgroundFilterParams& p) { params_ = p; }

    /**
     * @brief 배경 맵과 학습 카운터를 초기화한다 (SIGUSR1 현장 리셋용).
     *        호출 후 learning_frames 동안 배경을 다시 학습한다.
     */
    void Reset() {
        background_map_.clear();
        long_term_map_.clear();
        protected_positions_.clear();
        frame_count_ = 0;
    }

    void SetProtectedForegroundPositions(const std::vector<std::pair<double,double>>& positions) {
        protected_positions_ = positions;
    }

    void SetDumpedPositions(const std::vector<std::pair<double,double>>& positions) {
        SetProtectedForegroundPositions(positions);
    }

private:
    BackgroundFilterParams        params_;
    std::vector<BackgroundEntry>  background_map_;       // 단기 배경
    std::vector<BackgroundEntry>  long_term_map_;        // 장기 배경
    uint32_t                      frame_count_ = 0;
    std::vector<std::pair<double,double>> protected_positions_;
    std::vector<Cluster>          scratch_observed_clusters_;

    void LearnFrame(const std::vector<Cluster>& clusters);
    void FilterBackground(std::vector<Cluster>& clusters);
    void AdaptiveUpdate(const std::vector<Cluster>& observed_clusters,
                        const std::vector<Cluster>& foreground_clusters);

    bool IsInLongTermBg(double x, double y) const;
};

} // namespace ecowarden
