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
 * @file  background_filter.cpp
 * @brief 배경 학습 + 적응형 갱신 기반 고정 물체 필터링 구현
 * @date  2026
 */

#include "background_filter.h"

#include <cmath>
#include <cstdio>
#include <algorithm>

namespace ecowarden {

// ── 생성자 ──────────────────────────────────────────────────────────
BackgroundFilter::BackgroundFilter(const BackgroundFilterParams& params)
    : params_(params) {}

// ── Apply: 학습/운용 분기 ───────────────────────────────────────────
bool BackgroundFilter::Apply(std::vector<Cluster>& clusters) {
    if (IsLearning()) {
        LearnFrame(clusters);
        frame_count_++;

        if (frame_count_ == params_.learning_frames) {
            // 학습 완료 → 충분히 관측된 엔트리만 확정 배경으로 마킹
            // 학습 프레임의 30% 이상에서 관측되어야 안정된 배경으로 간주
            uint32_t min_seen = std::max(1u, params_.learning_frames * 3 / 10);
            background_map_.erase(
                std::remove_if(background_map_.begin(), background_map_.end(),
                    [min_seen](const BackgroundEntry& bg) {
                        return bg.seen_count < min_seen;
                    }),
                background_map_.end()
            );
            for (auto& bg : background_map_) {
                bg.confirmed = true;
            }
            // 장기 배경에도 동일하게 초기화
            long_term_map_ = background_map_;
            std::printf("[INFO] 배경 학습 완료. 등록된 고정 물체 수: %zu\n",
                        background_map_.size());
        }
        return false;
    }

    scratch_observed_clusters_ = clusters;
    FilterBackground(clusters);

    // 적응형 배경 갱신 (필터링 후 남은 클러스터 기반)
    if (params_.enable_adaptive) {
        AdaptiveUpdate(scratch_observed_clusters_, clusters);
    }

    frame_count_++;
    return true;
}

// ── 학습: 클러스터를 배경 맵에 누적 ─────────────────────────────────
void BackgroundFilter::LearnFrame(const std::vector<Cluster>& clusters) {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;

    for (const auto& cl : clusters) {
        // 너무 작은 노이즈성 클러스터는 배경 학습에서 제외 (최소 50mm 이상)
        if (cl.width_mm < 50.0) continue;

        bool matched = false;

        for (auto& bg : background_map_) {
            double dx = cl.centroid_x_mm - bg.x_mm;
            double dy = cl.centroid_y_mm - bg.y_mm;

            if (dx * dx + dy * dy <= radius_sq) {
                // 이동 평균으로 배경 위치 정밀화
                bg.x_mm = (bg.x_mm * 0.9) + (cl.centroid_x_mm * 0.1);
                bg.y_mm = (bg.y_mm * 0.9) + (cl.centroid_y_mm * 0.1);
                bg.seen_count++;
                matched = true;
                break;
            }
        }

        if (!matched) {
            BackgroundEntry entry;
            entry.x_mm       = cl.centroid_x_mm;
            entry.y_mm       = cl.centroid_y_mm;
            entry.width_mm   = cl.width_mm;
            entry.seen_count  = 1;
            entry.absent_count = 0;
            entry.confirmed  = false;
            background_map_.push_back(entry);
        }
    }
}

// ── 운용: 배경 맵에 매칭되는 클러스터 제거 ──────────────────────────
//   개선: 배경 영역을 지날 때 점이 생기는 현상을 방지하기 위해 
//   단기 배경 매칭 시 우선적으로 제거하되, 장기 배경 모델은 '신규 투기물' 판단에만 활용
void BackgroundFilter::FilterBackground(std::vector<Cluster>& clusters) {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;
    const double residual_radius_sq =
        params_.residual_filter_radius_mm * params_.residual_filter_radius_mm;
    const double protect_radius_mm =
        std::max(params_.match_radius_mm, params_.residual_filter_radius_mm);
    const double protect_radius_sq = protect_radius_mm * protect_radius_mm;
    // 잔상 방지용 여유 반경 — 매우 작은 노이즈 점만 잡도록 축소 (투기물 오필터 방지)
    const double slack_radius_sq = (params_.match_radius_mm + 30.0) * (params_.match_radius_mm + 30.0);

    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
            [&](const Cluster& cl) {
                // 추적 중인 객체 위치와 가까운 클러스터는 배경 필터링에서 보호
                for (const auto& dp : protected_positions_) {
                    double ddx = cl.centroid_x_mm - dp.first;
                    double ddy = cl.centroid_y_mm - dp.second;
                    if (ddx * ddx + ddy * ddy <= protect_radius_sq) {
                        return false;
                    }
                }

                if (params_.residual_filter_after_frames > 0 &&
                    cl.width_mm <= params_.residual_filter_max_width_mm) {
                    for (const auto& bg : background_map_) {
                        if (bg.confirmed ||
                            bg.seen_count < params_.residual_filter_after_frames) {
                            continue;
                        }
                        double dx = cl.centroid_x_mm - bg.x_mm;
                        double dy = cl.centroid_y_mm - bg.y_mm;
                        if (dx * dx + dy * dy <= residual_radius_sq) {
                            return true;
                        }
                    }
                }

                bool in_short_term = false;
                for (auto& bg : background_map_) {
                    if (!bg.confirmed) continue;

                    double dx = cl.centroid_x_mm - bg.x_mm;
                    double dy = cl.centroid_y_mm - bg.y_mm;
                    double d_sq = dx * dx + dy * dy;

                    // 1. 표준 반경 내에 있으면 배경
                    if (d_sq <= radius_sq) {
                        bg.absent_count = 0;
                        in_short_term = true;
                        break;
                    }

                    // 2. 여유 반경 내의 매우 작은 점(잔상)만 배경으로 간주
                    if (d_sq <= slack_radius_sq && cl.width_mm < 50.0) {
                        bg.absent_count = 0;
                        in_short_term = true;
                        break;
                    }
                }

                if (!in_short_term) return false;

                // 이중 배경 모델 처리 (투기물 보호 로직)
                if (params_.enable_dual_background) {
                    bool in_long_term = IsInLongTermBg(cl.centroid_x_mm, cl.centroid_y_mm);
                    if (!in_long_term) {
                        // 단기 배경에만 있고 장기 배경엔 없음 = 새로 놓인 물체 → 전경으로 남김
                        return false;
                    }
                }

                return true;
            }),
        clusters.end()
    );
}

// -- 장기 배경 매칭 검사 --
bool BackgroundFilter::IsInLongTermBg(double x, double y) const {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;
    for (const auto& bg : long_term_map_) {
        if (!bg.confirmed) continue;
        double dx = x - bg.x_mm;
        double dy = y - bg.y_mm;
        if (dx * dx + dy * dy <= radius_sq) return true;
    }
    return false;
}

// ── 적응형 배경 갱신 ────────────────────────────────────────────────
//
//   운용 단계에서:
//     1. 기존 배경 엔트리가 현재 프레임에서 관측되면 absent 리셋
//     2. 관측되지 않으면 absent_count 증가
//     3. absent_count > remove_after_absent → 배경에서 제거 (물체가 치워짐)
//     4. 필터링 후 남은 클러스터(전경) 중 배경 근처에 계속 있는 것 → 새 배경 후보
//        seen_count > add_after_frames → 배경 등록 (물체가 새로 놓임)
//
void BackgroundFilter::AdaptiveUpdate(const std::vector<Cluster>& observed_clusters,
                                      const std::vector<Cluster>& foreground_clusters) {
    const double radius_sq = params_.match_radius_mm * params_.match_radius_mm;

    // 기존 배경 관측 여부 체크 — 필터링에서 제거된 클러스터는 여기 안 오므로
    // 원본 스캔의 전체 클러스터가 필요하지만, 성능상 간접 추정:
    // "배경 위치에 뭔가 있는지"를 전경 클러스터 기준으로만 판단하면 부정확.
    // 대신: 매 프레임마다 absent_count를 1씩 증가시키고,
    //        FilterBackground에서 매칭된 경우 리셋하는 방식 사용.

    // 1) 모든 배경 엔트리의 absent_count 증가
    for (auto& bg : background_map_) {
        bg.absent_count++;
    }

    for (const auto& cl : observed_clusters) {
        for (auto& bg : background_map_) {
            if (!bg.confirmed) continue;
            double dx = cl.centroid_x_mm - bg.x_mm;
            double dy = cl.centroid_y_mm - bg.y_mm;
            if (dx * dx + dy * dy <= radius_sq) {
                bg.absent_count = 0;
                break;
            }
        }
    }

    // 2) absent_count > 제한 → 제거
    background_map_.erase(
        std::remove_if(background_map_.begin(), background_map_.end(),
            [this](const BackgroundEntry& bg) {
                return bg.absent_count > params_.remove_after_absent;
            }),
        background_map_.end()
    );

    // 3) 전경 클러스터가 기존 배경 후보와 가까우면 seen_count 증가
    //    아니면 새 후보로 등록
    for (const auto& cl : foreground_clusters) {
        bool matched_candidate = false;

        for (auto& bg : background_map_) {
            if (bg.confirmed) continue;  // 이미 확정된 배경은 건너뜀

            double dx = cl.centroid_x_mm - bg.x_mm;
            double dy = cl.centroid_y_mm - bg.y_mm;
            if (dx * dx + dy * dy <= radius_sq) {
                bg.seen_count++;
                bg.x_mm = (bg.x_mm + cl.centroid_x_mm) / 2.0;
                bg.y_mm = (bg.y_mm + cl.centroid_y_mm) / 2.0;
                matched_candidate = true;

                // seen_count 임계값 초과 → 확정 배경 승격
                // 단, 투기 의심/확정 물체 위치는 배경 등록에서 제외
                if (bg.seen_count >= params_.add_after_frames) {
                    bool near_dump = false;
                    for (const auto& dp : protected_positions_) {
                        double ddx = bg.x_mm - dp.first;
                        double ddy = bg.y_mm - dp.second;
                        if (ddx * ddx + ddy * ddy <= radius_sq) {
                            near_dump = true;
                            break;
                        }
                    }
                    if (near_dump) {
                        bg.seen_count = 0;
                        break;
                    }
                    bg.confirmed = true;
                    std::printf("[INFO] 적응형 배경 등록: (%.0f, %.0f) mm\n",
                                bg.x_mm, bg.y_mm);
                }
                break;
            }
        }

        if (!matched_candidate) {
            // 새 후보 등록 (아직 배경이 아님 — 필터링에 영향 없음)
            BackgroundEntry entry;
            entry.x_mm        = cl.centroid_x_mm;
            entry.y_mm        = cl.centroid_y_mm;
            entry.width_mm    = cl.width_mm;
            entry.seen_count   = 1;
            entry.absent_count = 0;
            entry.confirmed   = false;
            background_map_.push_back(entry);
        }
    }

    // ── 장기 배경 모델 독립 갱신 ─────────────────────────────────────
    if (params_.enable_dual_background) {
        // absent_count 증가
        for (auto& bg : long_term_map_) { bg.absent_count++; }

        for (const auto& cl : observed_clusters) {
            for (auto& bg : long_term_map_) {
                if (!bg.confirmed) continue;
                double dx = cl.centroid_x_mm - bg.x_mm;
                double dy = cl.centroid_y_mm - bg.y_mm;
                if (dx * dx + dy * dy <= radius_sq) {
                    bg.absent_count = 0;
                    break;
                }
            }
        }

        // 오래 안 보이면 제거
        long_term_map_.erase(
            std::remove_if(long_term_map_.begin(), long_term_map_.end(),
                [this](const BackgroundEntry& bg) {
                    return bg.absent_count > params_.long_term_remove_after;
                }),
            long_term_map_.end()
        );

        // 전경 클러스터를 장기 모델에도 등록 시도
        for (const auto& cl : foreground_clusters) {
            bool matched_lt = false;
            for (auto& bg : long_term_map_) {
                double dx = cl.centroid_x_mm - bg.x_mm;
                double dy = cl.centroid_y_mm - bg.y_mm;
                if (dx * dx + dy * dy <= radius_sq) {
                    bg.seen_count++;
                    bg.absent_count = 0;
                    if (!bg.confirmed &&
                        bg.seen_count >= params_.long_term_add_frames) {
                        bg.confirmed = true;
                        std::printf("[INFO] 장기 배경 등록: (%.0f, %.0f) mm\n",
                                    bg.x_mm, bg.y_mm);
                    }
                    matched_lt = true;
                    break;
                }
            }
            if (!matched_lt) {
                BackgroundEntry entry;
                entry.x_mm        = cl.centroid_x_mm;
                entry.y_mm        = cl.centroid_y_mm;
                entry.width_mm    = cl.width_mm;
                entry.seen_count   = 1;
                entry.absent_count = 0;
                entry.confirmed   = false;
                long_term_map_.push_back(entry);
            }
        }
    }
}

} // namespace ecowarden
