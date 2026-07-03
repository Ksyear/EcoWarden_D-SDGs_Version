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
 * @file  scan_processor.cpp
 * @brief 노이즈 필터 + 극좌표→직교좌표 변환 + DBSCAN 클러스터링 구현
 * @date  2026
 */

#include "scan_processor.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

namespace ecowarden {

// ── 생성자 ──────────────────────────────────────────────────────────
ScanProcessor::ScanProcessor(const FilterParams& filter, const DBSCANParams& dbscan)
    : filter_(filter), dbscan_(dbscan) {}

// ── 전체 파이프라인 ─────────────────────────────────────────────────
size_t ScanProcessor::Process(const ScanFrame& raw, std::vector<Cluster>& clusters) {
    // 1) 노이즈 필터
    ScanFrame filtered;
    FilterNoise(raw, filtered);

    if (filtered.empty()) {
        clusters.clear();
        return 0;
    }

    // 1.5) 정적 배경 필터 적용 (벽면 밀착 객체 분리용)
    ScanFrame bg_filtered;
    FilterStaticBackground(filtered, bg_filtered);

    if (bg_filtered.empty()) {
        clusters.clear();
        return 0;
    }

    // 2) 극좌표 → 직교좌표
    std::vector<CartesianPoint> cart;
    ToCartesian(bg_filtered, cart);

    // 3) DBSCAN 클러스터링
    ClusterDBSCAN(cart, clusters);

    // 4) 근접 클러스터 병합 (보행자 다리 분리 방지)
    if (filter_.merge_radius_mm > 0.0) {
        MergeClusters(clusters);
    }

    return bg_filtered.size(); // 실제 활용되는 포인트 수 반환
}

// ── 노이즈 필터 ─────────────────────────────────────────────────────
//   거리 0~30mm: 센서 근접 노이즈 제거
//   거리 12000mm 초과: RplidarS2 최대 측정거리 초과 → 무효
void ScanProcessor::FilterNoise(const ScanFrame& raw, ScanFrame& filtered) {
    filtered.clear();
    filtered.reserve(raw.size());

    for (const auto& pt : raw) {
        if (pt.intensity < filter_.min_intensity)            continue;
        if (pt.distance_mm <= filter_.min_distance_mm ||
            pt.distance_mm >  filter_.max_distance_mm)       continue;
        if (pt.angle_deg < filter_.fov_min_deg ||
            pt.angle_deg > filter_.fov_max_deg)              continue;
        filtered.push_back(pt);
    }
}

int ScanProcessor::NormalizeSlot(int idx) {
    idx %= 3600;
    if (idx < 0) idx += 3600;
    return idx;
}

float ScanProcessor::ProcessingAngleDeg(float raw_angle_deg) const {
    if (!filter_.reverse_angle) return raw_angle_deg;

    float angle = 360.0f - raw_angle_deg;
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

bool ScanProcessor::HasStaticForegroundNeighbor(int idx,
                                                uint16_t distance_mm) const {
    const int guard = static_cast<int>(filter_.static_bg_edge_guard_slots);
    if (guard <= 0) return true;

    for (int d = -guard; d <= guard; ++d) {
        if (d == 0) continue;
        const int ni = NormalizeSlot(idx + d);
        const uint16_t bg_dist = static_bg_depth_[ni];
        if (bg_dist == 0) continue;
        if (static_bg_confidence_[ni] < filter_.static_bg_min_confidence) continue;
        if (distance_mm < bg_dist &&
            (bg_dist - distance_mm) > filter_.static_bg_margin_mm) {
            return true;
        }
    }
    return false;
}

// ── 정적 배경 학습 ──────────────────────────────────────────────────
void ScanProcessor::LearnStaticBackground(const ScanFrame& raw,
                                           const bool *blocked_slots_3600) {
    for (const auto& pt : raw) {
        if (pt.distance_mm == 0) continue;

        int idx = NormalizeSlot(static_cast<int>(
            std::round(ProcessingAngleDeg(pt.angle_deg) * 10.0)));

        if (blocked_slots_3600) {
            bool blocked = false;
            const int guard = static_cast<int>(filter_.static_bg_edge_guard_slots);
            for (int d = -guard; d <= guard; ++d) {
                if (blocked_slots_3600[NormalizeSlot(idx + d)]) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
        }

        static_bg_prev_depth_[idx] = static_bg_depth_[idx];
        if (static_bg_depth_[idx] == 0) {
            static_bg_depth_[idx] = pt.distance_mm;
        } else {
            const double alpha = std::clamp(static_cast<double>(filter_.static_bg_ema_alpha),
                                            0.0, 1.0);
            const double updated =
                static_bg_depth_[idx] * (1.0 - alpha) + pt.distance_mm * alpha;
            static_bg_depth_[idx] = static_cast<uint16_t>(std::round(updated));
        }
        if (static_bg_confidence_[idx] < 255) {
            static_bg_confidence_[idx]++;
        }
    }
}

// ── 정적 배경 필터링 ────────────────────────────────────────────────
void ScanProcessor::FilterStaticBackground(const ScanFrame& raw, ScanFrame& filtered) {
    filtered.clear();
    filtered.reserve(raw.size());

    for (const auto& pt : raw) {
        int idx = NormalizeSlot(static_cast<int>(
            std::round(ProcessingAngleDeg(pt.angle_deg) * 10.0)));
        
        uint16_t bg_dist = static_bg_depth_[idx];
        
        // 해당 슬롯이 학습 안된 곳이면 (0이면) 일단 통과시킴
        if (bg_dist == 0) {
            filtered.push_back(pt);
            continue;
        }

        if (static_bg_confidence_[idx] < filter_.static_bg_min_confidence) {
            continue;
        }
        
        // 측정 거리가 배경보다 설정된 margin 이상으로 가까우면 = 앞으로 튀어나온 객체 = 전경
        // 측정 거리가 배경과 비슷하면 = 벽면
        // 측정 거리가 배경보다 멀리 측정되면 = 유리창 반사 등의 노이즈
        if (pt.distance_mm < bg_dist) {
            if ((bg_dist - pt.distance_mm) > filter_.static_bg_margin_mm) {
                const uint16_t prev = static_bg_prev_depth_[idx];
                const bool jump_once =
                    prev > 0 &&
                    std::abs(static_cast<int>(pt.distance_mm) -
                             static_cast<int>(prev)) >
                        static_cast<int>(filter_.static_bg_jump_reject_mm) &&
                    !HasStaticForegroundNeighbor(idx, pt.distance_mm);
                if (jump_once) continue;
                if (!HasStaticForegroundNeighbor(idx, pt.distance_mm)) continue;
                filtered.push_back(pt);
            }
        }
    }
}

// ── 극좌표 → 직교좌표 변환 ──────────────────────────────────────────
//   LiDAR 기준 좌표계:
//     X = distance * cos(angle)    (전방이 0°)
//     Y = distance * sin(angle)    (반시계 방향 +)
void ScanProcessor::ToCartesian(const ScanFrame& polar,
                                std::vector<CartesianPoint>& cart) {
    cart.clear();
    cart.reserve(polar.size());

    for (const auto& pt : polar) {
        const float angle_deg = ProcessingAngleDeg(pt.angle_deg);
        double rad = angle_deg * kDegToRad;
        cart.push_back(CartesianPoint{
            .x_mm        = pt.distance_mm * std::cos(rad),
            .y_mm        = pt.distance_mm * std::sin(rad),
            .angle_deg   = angle_deg,
            .distance_mm = pt.distance_mm,
            .intensity   = pt.intensity,
        });
    }
}

// ── 그리드 인덱스 구축 ──────────────────────────────────────────────
//   scratch_grid_ 를 재사용한다 — clear() 는 bucket 배열을 유지하므로
//   매 프레임 rehash 비용이 사라진다. 각 셀 벡터도 clear() 로만 초기화.
void ScanProcessor::BuildGrid(const std::vector<CartesianPoint>& points,
                              double cell_size) {
    for (auto& kv : scratch_grid_) {
        kv.second.clear();
    }
    for (size_t i = 0; i < points.size(); ++i) {
        GridKey key{
            static_cast<int32_t>(std::floor(points[i].x_mm / cell_size)),
            static_cast<int32_t>(std::floor(points[i].y_mm / cell_size))
        };
        scratch_grid_[key].push_back(i);
    }
}

// ── RangeQueryGrid: 그리드 가속 eps 반경 이웃 검색 ──────────────────
//   cell_size = epsilon 이므로 자신 + 주변 8셀만 탐색 → O(1) 평균
void ScanProcessor::RangeQueryGrid(
    const std::vector<CartesianPoint>& points,
    double cell_size,
    size_t idx,
    double eps_sq,
    std::vector<size_t>& neighbors)
{
    neighbors.clear();
    const double px = points[idx].x_mm;
    const double py = points[idx].y_mm;
    const int32_t gx = static_cast<int32_t>(std::floor(px / cell_size));
    const int32_t gy = static_cast<int32_t>(std::floor(py / cell_size));

    for (int32_t dx = -1; dx <= 1; ++dx) {
        for (int32_t dy = -1; dy <= 1; ++dy) {
            auto it = scratch_grid_.find(GridKey{gx + dx, gy + dy});
            if (it == scratch_grid_.end()) continue;

            for (size_t i : it->second) {
                double ddx = points[i].x_mm - px;
                double ddy = points[i].y_mm - py;
                if (ddx * ddx + ddy * ddy <= eps_sq) {
                    neighbors.push_back(i);
                }
            }
        }
    }
}

// ── DBSCAN 클러스터링 ───────────────────────────────────────────────
//   epsilon  = 100mm  (이웃 탐색 반경, production_params.h 참조)
//   minPts   = 5      (코어 포인트 조건)
//
//   알고리즘:
//   1. 모든 포인트를 UNVISITED로 초기화
//   2. 각 포인트에 대해 eps 반경 내 이웃 검색
//   3. 이웃 수 >= minPts → 새 클러스터 생성, 이웃 확장
//   4. 이웃 수 < minPts → NOISE로 마킹 (나중에 다른 클러스터에 편입 가능)
void ScanProcessor::ClusterDBSCAN(const std::vector<CartesianPoint>& points,
                                  std::vector<Cluster>& clusters) {
    clusters.clear();

    const size_t n = points.size();
    if (n == 0) return;

    const double eps_sq = dbscan_.epsilon_mm * dbscan_.epsilon_mm;
    const double cell_size = dbscan_.epsilon_mm;  // 그리드 셀 크기 = epsilon

    // 그리드 인덱스 구축 (O(n)) — 영속 scratch_grid_ 재사용
    BuildGrid(points, cell_size);

    // 레이블: UNVISITED(-1), NOISE(-2), 또는 클러스터 ID (0, 1, 2, ...)
    //   scratch_labels_ 는 멤버 버퍼 — assign 으로 재사용.
    scratch_labels_.assign(n, UNVISITED);
    int cluster_id = 0;

    // 이웃 버퍼도 멤버 — 각 RangeQueryGrid 호출이 clear 후 채운다.
    std::vector<size_t>& neighbors     = scratch_neighbors_;
    std::vector<size_t>& sub_neighbors = scratch_sub_neighbors_;

    for (size_t i = 0; i < n; ++i) {
        if (scratch_labels_[i] != UNVISITED) continue;

        RangeQueryGrid(points, cell_size, i, eps_sq, neighbors);

        if (neighbors.size() < dbscan_.min_points) {
            scratch_labels_[i] = NOISE;
            continue;
        }

        int cid = cluster_id++;
        scratch_labels_[i] = cid;

        for (size_t q = 0; q < neighbors.size(); ++q) {
            size_t nb = neighbors[q];

            if (scratch_labels_[nb] == NOISE) {
                scratch_labels_[nb] = cid;
                continue;
            }
            if (scratch_labels_[nb] != UNVISITED) continue;

            scratch_labels_[nb] = cid;

            RangeQueryGrid(points, cell_size, nb, eps_sq, sub_neighbors);

            if (sub_neighbors.size() >= dbscan_.min_points) {
                for (size_t sn : sub_neighbors) {
                    if (scratch_labels_[sn] == UNVISITED || scratch_labels_[sn] == NOISE) {
                        neighbors.push_back(sn);
                    }
                }
            }
        }
    }

    // ── 레이블 → Cluster 구조체 변환 ────────────────────────────────
    if (cluster_id == 0) return;

    clusters.resize(cluster_id);
    for (int c = 0; c < cluster_id; ++c) {
        clusters[c].centroid_x_mm = 0.0;
        clusters[c].centroid_y_mm = 0.0;
    }

    for (size_t i = 0; i < n; ++i) {
        int lbl = scratch_labels_[i];
        if (lbl < 0) continue; // NOISE는 어느 클러스터에도 속하지 않음

        clusters[lbl].points.push_back(points[i]);
    }

    // 중심점(centroid) 계산
    for (auto& cl : clusters) {
        if (cl.points.empty()) continue;

        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& pt : cl.points) {
            sum_x += pt.x_mm;
            sum_y += pt.y_mm;
        }
        cl.centroid_x_mm = sum_x / cl.points.size();
        cl.centroid_y_mm = sum_y / cl.points.size();
    }

    // 빈 클러스터 제거 (border point만으로 구성된 경우는 드물지만 방어)
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [](const Cluster& c) { return c.points.empty(); }),
        clusters.end()
    );

    // 클러스터 폭(width) 계산: AABB 기반 O(n) 근사 (기존 O(n²) → O(n))
    // 실제 최대 점간 거리와 AABB 대각선 오차는 무시할 수준 (LiDAR 해상도 대비)
    for (auto& cl : clusters) {
        if (cl.points.size() < 2) {
            cl.width_mm = 0.0;
            continue;
        }
        double min_x = cl.points[0].x_mm, max_x = min_x;
        double min_y = cl.points[0].y_mm, max_y = min_y;
        for (size_t i = 1; i < cl.points.size(); ++i) {
            if (cl.points[i].x_mm < min_x) min_x = cl.points[i].x_mm;
            if (cl.points[i].x_mm > max_x) max_x = cl.points[i].x_mm;
            if (cl.points[i].y_mm < min_y) min_y = cl.points[i].y_mm;
            if (cl.points[i].y_mm > max_y) max_y = cl.points[i].y_mm;
        }
        double dx = max_x - min_x;
        double dy = max_y - min_y;
        cl.width_mm = std::sqrt(dx * dx + dy * dy);
    }

    // 폭 기반 필터링
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [this](const Cluster& c) {
                           return c.width_mm < filter_.min_cluster_width_mm ||
                                  c.width_mm > filter_.max_cluster_width_mm;
                       }),
        clusters.end()
    );

    // 포인트 수 기준 내림차순 정렬 (가장 큰 클러스터 먼저)
    std::sort(clusters.begin(), clusters.end(),
              [](const Cluster& a, const Cluster& b) {
                  return a.points.size() > b.points.size();
              });
}

// ── 근접 클러스터 병합 ──────────────────────────────────────────────
//
//   보행자의 두 다리가 DBSCAN에서 별도 클러스터로 분리되는 문제 해결:
//     - 성인 보행 시 다리 간격: 약 150~300mm
//     - merge_radius_mm (예: 300mm) 이내 centroid를 가진 클러스터를 병합
//
//   알고리즘:
//     1. Union-Find로 merge_radius 이내 클러스터 쌍을 그룹화
//     2. 같은 그룹의 클러스터를 하나로 합침 (포인트 통합 + centroid/width 재계산)
//
void ScanProcessor::MergeClusters(std::vector<Cluster>& clusters) {
    const size_t n = clusters.size();
    if (n < 2) return;

    const double merge_sq = filter_.merge_radius_mm * filter_.merge_radius_mm;
    const double leg_pair_merge_sq =
        filter_.leg_pair_merge_radius_mm * filter_.leg_pair_merge_radius_mm;

    auto is_leg_sized = [this](const Cluster& cl) {
        return cl.width_mm <= filter_.leg_pair_max_width_mm &&
               cl.points.size() <= filter_.leg_pair_max_points;
    };

    // Union-Find
    std::vector<size_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0);

    std::function<size_t(size_t)> find = [&](size_t x) -> size_t {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    auto unite = [&](size_t a, size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[b] = a;
    };

    // centroid 간 거리가 merge radius 이내이고 폭/점수 조건을 만족하는 쌍만 병합
    // (폭 비율 체크: 사람 다리끼리는 비슷한 폭 → 병합 OK,
    //  사람+봉투는 폭 차이 큼 → 병합 차단)
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double dx = clusters[i].centroid_x_mm - clusters[j].centroid_x_mm;
            double dy = clusters[i].centroid_y_mm - clusters[j].centroid_y_mm;
            double d2 = dx * dx + dy * dy;

            bool normal_merge = d2 <= merge_sq;
            bool leg_pair_merge =
                filter_.leg_pair_merge_radius_mm > 0.0 &&
                d2 <= leg_pair_merge_sq &&
                is_leg_sized(clusters[i]) &&
                is_leg_sized(clusters[j]);

            if (!normal_merge && !leg_pair_merge) continue;

            double w1 = clusters[i].width_mm;
            double w2 = clusters[j].width_mm;
            if (w1 <= 0.0 || w2 <= 0.0) continue;

            double ratio = (w1 > w2) ? w1 / w2 : w2 / w1;
            if (ratio > filter_.merge_max_width_ratio) continue;

            if (normal_merge || leg_pair_merge) {
                unite(i, j);
            }
        }
    }

    // 그룹 ID → 인덱스 매핑으로 포인트를 모아 새 클러스터 생성
    std::vector<Cluster> merged;
    std::vector<int> group_map(n, -1);

    for (size_t i = 0; i < n; ++i) {
        size_t root = find(i);

        if (group_map[root] < 0) {
            group_map[root] = static_cast<int>(merged.size());
            merged.push_back(Cluster{});
        }

        int mi = group_map[root];
        merged[mi].points.insert(merged[mi].points.end(),
                                 clusters[i].points.begin(),
                                 clusters[i].points.end());
    }

    // centroid + width 재계산
    for (auto& cl : merged) {
        if (cl.points.empty()) continue;

        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& pt : cl.points) {
            sum_x += pt.x_mm;
            sum_y += pt.y_mm;
        }
        cl.centroid_x_mm = sum_x / cl.points.size();
        cl.centroid_y_mm = sum_y / cl.points.size();

        if (cl.points.size() < 2) {
            cl.width_mm = 0.0;
        } else {
            double min_x = cl.points[0].x_mm, max_x = min_x;
            double min_y = cl.points[0].y_mm, max_y = min_y;
            for (size_t i = 1; i < cl.points.size(); ++i) {
                if (cl.points[i].x_mm < min_x) min_x = cl.points[i].x_mm;
                if (cl.points[i].x_mm > max_x) max_x = cl.points[i].x_mm;
                if (cl.points[i].y_mm < min_y) min_y = cl.points[i].y_mm;
                if (cl.points[i].y_mm > max_y) max_y = cl.points[i].y_mm;
            }
            double dwx = max_x - min_x;
            double dwy = max_y - min_y;
            cl.width_mm = std::sqrt(dwx * dwx + dwy * dwy);
        }
    }

    clusters = std::move(merged);
}

} // namespace ecowarden
