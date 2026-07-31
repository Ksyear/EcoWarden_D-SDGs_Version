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
 * @file  cluster_tracker.cpp
 * @brief 프레임 간 클러스터 추적 + 객체 이탈 감지 + 투기 감지 구현
 * @date  2026
 *
 * 알고리즘 요약:
 *   1. 그리디 최근접 매칭으로 현재 클러스터를 기존 트랙에 연결
 *   2. 매칭된 트랙의 centroid 이동거리로 이동/정지 판정 + 누적 이동거리 갱신
 *   3. "이동 이력 있음 + 연속 정지 ≥ departure_frame_count" → 이탈 이벤트
 *   4. 미매칭 클러스터 중 기존 트랙 이전 위치 근처에 나타난 것 → 투기 의심 물체
 *   5. 투기 의심 물체가 dump_stationary_frame_count 이상 정지 → 투기 확정
 *   6. 매칭 실패 트랙은 lost_age_limit 초과 시 삭제
 */

#include "cluster_tracker.h"
#include "dump_detector.h"
#include "time_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

namespace ecowarden {

// ── TrackState → 문자열 ─────────────────────────────────────────────
const char* TrackStateToString(TrackState s) {
    switch (s) {
        case TrackState::Tentative:  return "TENTATIVE";
        case TrackState::Moving:     return "MOVING";
        case TrackState::Stationary: return "STATIONARY";
        case TrackState::Departed:   return "DEPARTED";
        case TrackState::Lost:       return "LOST";
    }
    return "UNKNOWN";
}

// 현재 시각 헬퍼 — 과거엔 파일 로컬 중복이 있었으나 time_utils.h 로 통합.
uint64_t ClusterTracker::NowMs() { return ecowarden::NowMs(); }

// ── 생성자 ──────────────────────────────────────────────────────────
ClusterTracker::ClusterTracker(const TrackerParams& params)
    : params_(params) {}

// -- Hotspot 반경 내 여부 검사 --
bool ClusterTracker::IsInHotspot(double x, double y) const {
    const double r_sq = params_.hotspot_radius_mm * params_.hotspot_radius_mm;
    for (const auto& hp : hotspot_positions_) {
        double dx = x - hp.x_mm;
        double dy = y - hp.y_mm;
        if (dx * dx + dy * dy < r_sq) return true;
    }
    return false;
}

// ── 콜백 등록 ───────────────────────────────────────────────────────
void ClusterTracker::SetDepartureCallback(DepartureCallback cb) {
    dep_callback_ = std::move(cb);
}

void ClusterTracker::SetDumpingCallback(DumpingCallback cb) {
    dump_callback_ = std::move(cb);
}

void ClusterTracker::SetSuspectCallback(SuspectCallback cb) {
    suspect_callback_ = std::move(cb);
}

// ── 그리디 최근접 매칭 ──────────────────────────────────────────────
void ClusterTracker::AssociateGreedy(const std::vector<Cluster>& clusters)
{
    const size_t nc = clusters.size();
    const size_t nt = tracks_.size();

    // 영속 스크래치: 매 프레임 의미는 초기화되지만 capacity 는 유지된다.
    scratch_cluster_to_track_.assign(nc, -1);
    scratch_track_matched_.assign(nt, false);
    scratch_pairs_.clear();

    if (nc == 0 || nt == 0) return;

    const double max_dist_sq =
        params_.association_max_dist_mm * params_.association_max_dist_mm;

    for (size_t c = 0; c < nc; ++c) {
        for (size_t t = 0; t < nt; ++t) {
            double dx = clusters[c].centroid_x_mm - tracks_[t].x_mm;
            double dy = clusters[c].centroid_y_mm - tracks_[t].y_mm;
            double d2 = dx * dx + dy * dy;
            if (d2 <= max_dist_sq) {
                double score = d2;
                if (tracks_[t].width_mm > 0.0 && clusters[c].width_mm > 0.0) {
                    const double dw = std::fabs(clusters[c].width_mm - tracks_[t].width_mm);
                    const double penalty = dw * params_.association_width_penalty;
                    score += penalty * penalty;
                }
                scratch_pairs_.push_back({score, c, t});
            }
        }
    }

    std::sort(scratch_pairs_.begin(), scratch_pairs_.end(),
              [](const AssociatePair& a, const AssociatePair& b) {
                  return a.dist_sq < b.dist_sq;
              });

    for (const auto& p : scratch_pairs_) {
        if (scratch_cluster_to_track_[p.ci] != -1) continue;
        if (scratch_track_matched_[p.ti])          continue;

        scratch_cluster_to_track_[p.ci] = static_cast<int>(p.ti);
        scratch_track_matched_[p.ti]    = true;
    }

    // -- 2nd pass: Kalman Filter fallback --
    //
    //   ClusterTracker::Update() 프롤로그(0.5단계)에서 모든 트랙의 KF에 대해
    //   이번 프레임용 Predict를 이미 수행했으므로, 여기서 읽는 kf.GetX()/GetY()는
    //   진짜 "이번 프레임의 예측 위치"다.
    //
    //   게이트는 두 값의 max로 적응형으로 결정한다:
    //     1) 사용자 설정 베이스 (kf_fallback_dist_mm)
    //     2) 9 × (P[0][0] + P[1][1]) ≈ 3σ 신뢰 영역
    //
    //   lost 프레임이 누적되면 P가 자라므로 게이트가 자동 확장된다.
    //   단, 현장 반사 spike와 오매칭되지 않도록 운영 상한을 둔다.
    //
    const double base_gate_sq =
        params_.kf_fallback_dist_mm * params_.kf_fallback_dist_mm;

    for (size_t t = 0; t < nt; ++t) {
        if (scratch_track_matched_[t]) continue;
        if (!tracks_[t].kf.IsInitialized()) continue;
        if (tracks_[t].lost_count > params_.recovery_max_lost_frames) continue;

        const double pred_x = tracks_[t].kf.GetX();
        const double pred_y = tracks_[t].kf.GetY();

        const double sigma_sq = tracks_[t].kf.PositionUncertaintySq();
        const double max_gate_sq =
            params_.kf_fallback_max_gate_mm * params_.kf_fallback_max_gate_mm;
        const double gate_sq  = std::min(std::max(base_gate_sq, 9.0 * sigma_sq),
                                         max_gate_sq);

        double best_d2 = gate_sq;
        int    best_c  = -1;

        for (size_t c = 0; c < nc; ++c) {
            if (scratch_cluster_to_track_[c] != -1) continue;
            if (tracks_[t].width_mm > 0.0 && clusters[c].width_mm > 0.0) {
                const double ratio = tracks_[t].width_mm > clusters[c].width_mm
                    ? tracks_[t].width_mm / clusters[c].width_mm
                    : clusters[c].width_mm / tracks_[t].width_mm;
                if (ratio > 3.0) continue;
            }
            double dx = clusters[c].centroid_x_mm - pred_x;
            double dy = clusters[c].centroid_y_mm - pred_y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_c  = static_cast<int>(c);
            }
        }

        if (best_c >= 0) {
            scratch_cluster_to_track_[best_c] = static_cast<int>(t);
            scratch_track_matched_[t] = true;
        }
    }
}

bool ClusterTracker::IsObjectCandidate(const Track& tr) const {
    if (tr.is_dump_suspect || tr.is_dumped_item) return true;
    if (!tr.confirmed || tr.lost_count > 0) return false;
    if (tr.last_person_group_id != 0 && tr.person_group_hold_count > 0) return false;
    const bool stationary_object_like =
        !tr.was_moving &&
        tr.stationary_count >= params_.object_candidate_min_stationary_frames &&
        tr.cumulative_dist_mm <= params_.stationary_track_max_cum_dist_mm;
    if (tr.width_mm > 0.0 && tr.width_mm <= params_.leg_track_max_width_mm) {
        const double rejoin_sq =
            params_.person_group_rejoin_radius_mm * params_.person_group_rejoin_radius_mm;
        for (const auto& ent : person_entities_) {
            if (ent.hold_count == 0 && ent.lost_count > params_.person_group_hold_frames) {
                continue;
            }
            const double dx = tr.x_mm - ent.x_mm;
            const double dy = tr.y_mm - ent.y_mm;
            if (dx * dx + dy * dy <= rejoin_sq && !stationary_object_like) return false;
        }
    }
    if (tr.was_moving && tr.age <= params_.min_age_for_dump + params_.person_group_hold_frames) {
        return false;
    }
    if (tr.width_mm < params_.new_track_min_width_mm ||
        tr.width_mm > params_.object_candidate_max_width_mm) {
        return false;
    }

    const double speed_sq = tr.vx_mm * tr.vx_mm + tr.vy_mm * tr.vy_mm;
    const bool stationary =
        tr.stationary_count >= params_.object_candidate_min_stationary_frames ||
        speed_sq < params_.leg_track_min_move_mm * params_.leg_track_min_move_mm;

    if (!stationary) return false;
    if (tr.was_moving && tr.cumulative_dist_mm >= params_.min_walk_dist_mm) {
        return false;
    }
    return tr.cumulative_dist_mm <= params_.stationary_track_max_cum_dist_mm;
}

bool ClusterTracker::IsLegCandidate(const Track& tr) const {
    if (!tr.confirmed || tr.lost_count > 0) return false;
    if (tr.is_dump_suspect || tr.is_dumped_item || tr.is_object_candidate) return false;
    if (tr.age < params_.leg_track_min_age) return false;
    if (tr.width_mm <= 0.0 || tr.width_mm > params_.leg_track_max_width_mm) return false;

    const double speed_sq = tr.vx_mm * tr.vx_mm + tr.vy_mm * tr.vy_mm;
    const double min_speed_sq =
        params_.leg_track_min_move_mm * params_.leg_track_min_move_mm;

    return speed_sq >= min_speed_sq ||
           (tr.was_moving && tr.cumulative_dist_mm >= params_.min_walk_dist_mm);
}

bool ClusterTracker::IsLegLikeForPerson(const Track& tr) const {
    if (tr.lost_count > 0) return false;
    if (tr.is_dump_suspect || tr.is_dumped_item) return false;
    if (tr.is_object_candidate && tr.last_person_group_id == 0) return false;
    if (tr.width_mm <= 0.0 || tr.width_mm > params_.leg_track_max_width_mm) return false;
    if (tr.age < 1) return false;
    if (tr.confirmed) return true;
    return tr.consecutive_match_count > 0 || tr.total_match_count > 0;
}

bool ClusterTracker::IsPersonPairCandidate(const Track& tr) const {
    if (tr.lost_count > 0) return false;
    if (tr.is_dump_suspect || tr.is_dumped_item) return false;
    if (tr.width_mm <= 0.0 || tr.width_mm > params_.leg_track_max_width_mm) return false;
    if (tr.age < 1) return false;

    // 느린 보행 다리는 순간적으로 object_candidate처럼 보일 수 있다.
    // 단, 완전 정지 소형 물체까지 사람쌍으로 묶지 않도록 최소 이동 이력은 요구한다.
    if (tr.is_object_candidate &&
        tr.last_person_group_id == 0 &&
        tr.cumulative_dist_mm < params_.leg_track_min_move_mm) {
        return false;
    }

    if (tr.confirmed) return true;
    return tr.consecutive_match_count > 0 || tr.total_match_count > 0;
}

bool ClusterTracker::HasDirectEvidenceForCluster(const Cluster& cluster) const {
    const double sep_sq =
        params_.separation_max_dist_mm * params_.separation_max_dist_mm;
    const double split_sq =
        params_.split_match_radius_mm * params_.split_match_radius_mm;

    for (const auto& tr : tracks_) {
        if (tr.is_dumped_item || tr.is_dump_suspect) continue;
        if (tr.width_drop_detected) {
            const double dx = cluster.centroid_x_mm - tr.width_drop_x_mm;
            const double dy = cluster.centroid_y_mm - tr.width_drop_y_mm;
            if (dx * dx + dy * dy <= sep_sq) return true;
        }
        if (tr.split_detected) {
            const double dx = cluster.centroid_x_mm - tr.split_x_mm;
            const double dy = cluster.centroid_y_mm - tr.split_y_mm;
            if (dx * dx + dy * dy <= split_sq) return true;
        }
    }
    return false;
}

bool ClusterTracker::ShouldHoldPendingCluster(const Cluster& cluster,
                                              bool direct_evidence) {
    const double match_sq =
        params_.pending_cluster_match_dist_mm * params_.pending_cluster_match_dist_mm;
    int best_idx = -1;
    double best_d2 = match_sq;

    for (size_t i = 0; i < pending_clusters_.size(); ++i) {
        const auto& pending = pending_clusters_[i];
        const double dx = cluster.centroid_x_mm - pending.x_mm;
        const double dy = cluster.centroid_y_mm - pending.y_mm;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = static_cast<int>(i);
        }
    }

    PendingCluster* pending = nullptr;
    if (best_idx >= 0) {
        pending = &pending_clusters_[best_idx];
        pending->x_mm = pending->x_mm * 0.7 + cluster.centroid_x_mm * 0.3;
        pending->y_mm = pending->y_mm * 0.7 + cluster.centroid_y_mm * 0.3;
        pending->width_mm = pending->width_mm * 0.7 + cluster.width_mm * 0.3;
        pending->points = static_cast<uint8_t>(
            std::min<size_t>(cluster.points.size(), 255));
        if (pending->seen_count < 255) pending->seen_count++;
        pending->missed_count = 0;
        pending->direct_evidence = pending->direct_evidence || direct_evidence;
    } else {
        if (pending_clusters_.size() >= params_.pending_cluster_max_count) {
            auto it = std::max_element(
                pending_clusters_.begin(), pending_clusters_.end(),
                [](const PendingCluster& a, const PendingCluster& b) {
                    if (a.missed_count != b.missed_count) {
                        return a.missed_count < b.missed_count;
                    }
                    return a.first_frame > b.first_frame;
                });
            if (it != pending_clusters_.end()) {
                *it = PendingCluster{};
                pending = &(*it);
            }
        } else {
            pending_clusters_.push_back(PendingCluster{});
            pending = &pending_clusters_.back();
        }

        pending->x_mm = cluster.centroid_x_mm;
        pending->y_mm = cluster.centroid_y_mm;
        pending->first_x_mm = cluster.centroid_x_mm;
        pending->first_y_mm = cluster.centroid_y_mm;
        pending->width_mm = cluster.width_mm;
        pending->points = static_cast<uint8_t>(
            std::min<size_t>(cluster.points.size(), 255));
        pending->seen_count = 1;
        pending->missed_count = 0;
        pending->first_frame = frame_count_;
        pending->direct_evidence = direct_evidence;
    }

    uint32_t required = pending->direct_evidence
        ? params_.pending_direct_confirm_frames
        : params_.pending_cluster_confirm_frames;
    if (!pending->direct_evidence && params_.pending_static_extra_frames > 0) {
        const double dx = pending->x_mm - pending->first_x_mm;
        const double dy = pending->y_mm - pending->first_y_mm;
        const double static_sq = params_.stationary_threshold_mm *
                                 params_.stationary_threshold_mm;
        if (dx * dx + dy * dy <= static_sq) {
            required += params_.pending_static_extra_frames;
        }
    }
    return pending->seen_count < required;
}

void ClusterTracker::PrunePendingClusters() {
    pending_clusters_.erase(
        std::remove_if(pending_clusters_.begin(), pending_clusters_.end(),
            [this](const PendingCluster& pending) {
                return pending.missed_count > params_.pending_cluster_missed_limit;
            }),
        pending_clusters_.end());
}

void ClusterTracker::UpdatePersonGroups() {
    for (auto& ent : person_entities_) {
        ent.lost_count++;
        if (ent.hold_count > 0) ent.hold_count--;
    }

    for (auto& tr : tracks_) {
        tr.person_group_id = 0;
        tr.person_group_primary = false;
        tr.person_group_x_mm = tr.x_mm;
        tr.person_group_y_mm = tr.y_mm;
        tr.person_group_width_mm = tr.width_mm;
        tr.is_object_candidate = IsObjectCandidate(tr);
        if (tr.person_group_hold_count > 0) {
            tr.person_group_hold_count--;
        }
    }

    const double pair_sq =
        params_.person_group_pair_radius_mm * params_.person_group_pair_radius_mm;
    const double wide_pair_sq =
        params_.person_group_wide_pair_radius_mm *
        params_.person_group_wide_pair_radius_mm;

    auto assign_group = [this](size_t i, size_t j, uint32_t group_id) {
        const double dx = tracks_[i].x_mm - tracks_[j].x_mm;
        const double dy = tracks_[i].y_mm - tracks_[j].y_mm;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double group_width =
            std::max(dist, std::max(tracks_[i].width_mm, tracks_[j].width_mm));
        const double group_x = (tracks_[i].x_mm + tracks_[j].x_mm) * 0.5;
        const double group_y = (tracks_[i].y_mm + tracks_[j].y_mm) * 0.5;
        const bool i_primary = tracks_[i].id <= tracks_[j].id;

        tracks_[i].person_group_id = group_id;
        tracks_[j].person_group_id = group_id;
        tracks_[i].is_object_candidate = false;
        tracks_[j].is_object_candidate = false;
        tracks_[i].person_group_primary = i_primary;
        tracks_[j].person_group_primary = !i_primary;
        tracks_[i].person_group_x_mm = group_x;
        tracks_[j].person_group_x_mm = group_x;
        tracks_[i].person_group_y_mm = group_y;
        tracks_[j].person_group_y_mm = group_y;
        tracks_[i].person_group_width_mm = group_width;
        tracks_[j].person_group_width_mm = group_width;
        tracks_[i].last_person_group_id = group_id;
        tracks_[j].last_person_group_id = group_id;

        auto it = std::find_if(person_entities_.begin(), person_entities_.end(),
            [group_id](const PersonEntity& ent) { return ent.id == group_id; });
        if (it == person_entities_.end()) {
            PersonEntity ent{};
            ent.id = group_id;
            person_entities_.push_back(ent);
            it = std::prev(person_entities_.end());
        }

        it->primary_track_id = i_primary ? tracks_[i].id : tracks_[j].id;
        it->secondary_track_id = i_primary ? tracks_[j].id : tracks_[i].id;
        it->vx_mm = group_x - it->x_mm;
        it->vy_mm = group_y - it->y_mm;
        it->x_mm = group_x;
        it->y_mm = group_y;
        it->width_mm = group_width;
        it->lost_count = 0;
        it->hold_count = params_.person_group_hold_frames;
        it->age++;
        it->confirmed = true;
    };

    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i].person_group_id != 0) continue;
        if (!IsPersonPairCandidate(tracks_[i])) continue;

        for (size_t j = i + 1; j < tracks_.size(); ++j) {
            if (tracks_[j].person_group_id != 0) continue;
            if (!IsPersonPairCandidate(tracks_[j])) continue;

            const double dx = tracks_[i].x_mm - tracks_[j].x_mm;
            const double dy = tracks_[i].y_mm - tracks_[j].y_mm;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq > wide_pair_sq) continue;

            const double dist = std::sqrt(dist_sq);
            const double group_width =
                std::max(dist, std::max(tracks_[i].width_mm, tracks_[j].width_mm));
            if (group_width > params_.person_group_new_pair_max_width_mm) {
                if (dist_sq > wide_pair_sq ||
                    tracks_[i].width_mm > params_.person_group_wide_pair_leg_width_mm ||
                    tracks_[j].width_mm > params_.person_group_wide_pair_leg_width_mm) {
                    continue;
                }
                if (std::fabs(tracks_[i].y_mm - tracks_[j].y_mm) >
                    params_.person_group_wide_pair_max_depth_gap_mm) {
                    continue;
                }
                const uint32_t birth_gap =
                    tracks_[i].first_seen_frame > tracks_[j].first_seen_frame
                        ? tracks_[i].first_seen_frame - tracks_[j].first_seen_frame
                        : tracks_[j].first_seen_frame - tracks_[i].first_seen_frame;
                if (birth_gap > params_.person_group_wide_pair_birth_gap_frames) {
                    continue;
                }

                bool crowded = false;
                for (size_t k = 0; k < tracks_.size(); ++k) {
                    if (k == i || k == j) continue;
                    if (tracks_[k].person_group_id != 0) continue;
                    if (!IsPersonPairCandidate(tracks_[k])) continue;
                    const double kdx_i = tracks_[k].x_mm - tracks_[i].x_mm;
                    const double kdy_i = tracks_[k].y_mm - tracks_[i].y_mm;
                    const double kdx_j = tracks_[k].x_mm - tracks_[j].x_mm;
                    const double kdy_j = tracks_[k].y_mm - tracks_[j].y_mm;
                    if (kdx_i * kdx_i + kdy_i * kdy_i <= wide_pair_sq ||
                        kdx_j * kdx_j + kdy_j * kdy_j <= wide_pair_sq) {
                        crowded = true;
                        break;
                    }
                }
                if (crowded) continue;
            }

            const double speed_i_sq =
                tracks_[i].vx_mm * tracks_[i].vx_mm + tracks_[i].vy_mm * tracks_[i].vy_mm;
            const double speed_j_sq =
                tracks_[j].vx_mm * tracks_[j].vx_mm + tracks_[j].vy_mm * tracks_[j].vy_mm;
            const double min_speed_sq =
                params_.leg_track_min_move_mm * params_.leg_track_min_move_mm;

            // 보행 중 양쪽 다리는 anti-phase로 움직일 수 있다. dot <= 0을
            // hard reject하면 보폭이 벌어지는 순간 group 생성이 실패한다.
            (void)speed_i_sq;
            (void)speed_j_sq;
            (void)min_speed_sq;

            const uint32_t group_id = std::min(tracks_[i].id, tracks_[j].id);
            assign_group(i, j, group_id);
            tracks_[i].person_group_hold_count = params_.person_group_hold_frames;
            tracks_[j].person_group_hold_count = params_.person_group_hold_frames;
            break;
        }
    }

    const double hold_width_sq =
        params_.person_group_rejoin_radius_mm * params_.person_group_rejoin_radius_mm;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i].person_group_id != 0) continue;
        if (tracks_[i].last_person_group_id == 0 ||
            tracks_[i].person_group_hold_count == 0) continue;
        if (!IsPersonPairCandidate(tracks_[i])) continue;

        for (size_t j = i + 1; j < tracks_.size(); ++j) {
            if (tracks_[j].person_group_id != 0) continue;
            if (tracks_[j].last_person_group_id != tracks_[i].last_person_group_id ||
                tracks_[j].person_group_hold_count == 0) continue;
            if (!IsPersonPairCandidate(tracks_[j])) continue;

            const double dx = tracks_[i].x_mm - tracks_[j].x_mm;
            const double dy = tracks_[i].y_mm - tracks_[j].y_mm;
            if (dx * dx + dy * dy > hold_width_sq) continue;

            assign_group(i, j, tracks_[i].last_person_group_id);
            break;
        }
    }

    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i].person_group_id != 0) continue;
        if (!IsLegLikeForPerson(tracks_[i])) continue;

        const bool held_leg =
            tracks_[i].last_person_group_id != 0 &&
            tracks_[i].person_group_hold_count > 0;
        const double attach_radius = held_leg
            ? params_.person_group_hold_attach_radius_mm
            : params_.person_group_attach_radius_mm;
        const double rejoin_sq = attach_radius * attach_radius;

        PersonEntity* best_ent = nullptr;
        double best_d2 = rejoin_sq;
        for (auto& ent : person_entities_) {
            if (ent.hold_count == 0 && ent.lost_count > params_.person_group_hold_frames) {
                continue;
            }
            if (held_leg && ent.id != tracks_[i].last_person_group_id) {
                continue;
            }
            const double dx = tracks_[i].x_mm - ent.x_mm;
            const double dy = tracks_[i].y_mm - ent.y_mm;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_ent = &ent;
            }
        }
        if (!best_ent) continue;

        tracks_[i].person_group_id = best_ent->id;
        tracks_[i].is_object_candidate = false;
        bool already_represented = false;
        for (const auto& tr : tracks_) {
            if (tr.id != tracks_[i].id && tr.person_group_id == best_ent->id) {
                already_represented = true;
                break;
            }
        }

        tracks_[i].person_group_primary = !already_represented;
        tracks_[i].person_group_x_mm = best_ent->x_mm;
        tracks_[i].person_group_y_mm = best_ent->y_mm;
        tracks_[i].person_group_width_mm =
            std::max(std::sqrt(best_d2), tracks_[i].width_mm);
        tracks_[i].last_person_group_id = best_ent->id;
        tracks_[i].person_group_hold_count = params_.person_group_hold_frames;

        best_ent->vx_mm = tracks_[i].x_mm - best_ent->x_mm;
        best_ent->vy_mm = tracks_[i].y_mm - best_ent->y_mm;
        best_ent->x_mm = best_ent->x_mm * 0.7 + tracks_[i].x_mm * 0.3;
        best_ent->y_mm = best_ent->y_mm * 0.7 + tracks_[i].y_mm * 0.3;
        best_ent->width_mm = tracks_[i].person_group_width_mm;
        best_ent->primary_track_id = tracks_[i].id;
        best_ent->secondary_track_id = 0;
        best_ent->lost_count = 0;
        best_ent->hold_count = params_.person_group_hold_frames;
        best_ent->age++;
        best_ent->confirmed = true;
    }

    person_entities_.erase(
        std::remove_if(person_entities_.begin(), person_entities_.end(),
            [this](const PersonEntity& ent) {
                return ent.lost_count > params_.person_group_hold_frames;
            }),
        person_entities_.end());
}

// DetectClusterSplit, DetectSeparation, CheckDumpingConfirmation,
// Phase 4 (suspect evaluation), Phase 6.5 (trajectory matching),
// suspect callback 발화 — 모두 DumpDetector 에 위임 (dump_detector.h).

// ── Update: 핵심 추적 루프 ──────────────────────────────────────────
void ClusterTracker::Update(const std::vector<Cluster>& clusters,
                            std::vector<DepartureEvent>& dep_events,
                            std::vector<DumpingEvent>& dump_events) {
    dep_events.clear();
    dump_events.clear();
    frame_count_++;

    // ── Hotspot TTL 감소 및 만료 제거 [v2.0] ─────────────────────────
    for (auto& hp : hotspot_positions_) {
        if (hp.frames_remaining > 0) hp.frames_remaining--;
    }
    hotspot_positions_.erase(
        std::remove_if(hotspot_positions_.begin(), hotspot_positions_.end(),
                        [](const auto& hp) { return hp.frames_remaining == 0; }),
        hotspot_positions_.end());

    // 매칭 전 기존 트랙 수 기록 (이후 추가된 트랙은 track_matched 범위 밖)
    const size_t original_track_count = tracks_.size();

    // ── 0.5단계: 프레임당 트랙당 정확히 1회 KF Predict ───────────────
    //   불변식: 매칭 코드(AssociateGreedy 등)가 KF state를 읽기 *전에*
    //   모든 초기화된 트랙에 대해 단 한 번 Predict를 수행한다.
    //   이렇게 해야 AssociateGreedy 2nd pass의 kf.GetX()가 진짜 "예측 위치"가 된다.
    //   Phase 2(matched)/Phase 3(unmatched)에서는 이제 Predict를 호출하지 않는다.
    for (auto& tr : tracks_) {
        if (tr.kf.IsInitialized()) {
            tr.kf.Predict();
        }
    }

    // ── 1단계: 매칭 ─────────────────────────────────────────────────
    //   결과는 scratch_cluster_to_track_ / scratch_track_matched_ 에 쓰인다.
    AssociateGreedy(clusters);

    // ── DumpDetector 컨텍스트 설정 ──────────────────────────────────
    //   투기 감지 전 과정을 DumpDetector 에 위임한다.
    auto alloc_fn = [](void* user) -> uint32_t {
        return static_cast<ClusterTracker*>(user)->AllocId();
    };
    DumpDetector::Context dd_ctx{};
    dd_ctx.params            = &params_;
    dd_ctx.tracks            = &tracks_;
    dd_ctx.hotspot_positions = &hotspot_positions_;
    dd_ctx.deleted_positions = &deleted_positions_;
    dd_ctx.cluster_to_track  = &scratch_cluster_to_track_;
    dd_ctx.track_matched     = &scratch_track_matched_;
    dd_ctx.cluster_claimed   = &scratch_cluster_claimed_;
    dd_ctx.alloc_id          = alloc_fn;
    dd_ctx.alloc_user        = this;
    dd_ctx.frame_count       = frame_count_;
    dd_ctx.dump_callback     = dump_callback_;
    dd_ctx.suspect_callback  = suspect_callback_;
    DumpDetector dd(dd_ctx);

    // ── 1.5단계: 클러스터 분열 감지 (1 트랙 → 2 클러스터) ───────────
    dd.DetectClusterSplit(clusters);

    const double thresh_sq =
        params_.stationary_threshold_mm * params_.stationary_threshold_mm;

    // ── 2단계: 매칭된 트랙 갱신 ─────────────────────────────────────
    for (size_t c = 0; c < clusters.size(); ++c) {
        int ti = scratch_cluster_to_track_[c];
        if (ti < 0) continue;

        Track& tr = tracks_[ti];
        tr.prev_x_mm = tr.x_mm;
        tr.prev_y_mm = tr.y_mm;
        tr.x_mm      = clusters[c].centroid_x_mm;
        tr.y_mm      = clusters[c].centroid_y_mm;

        // 폭 변화 추적 (투기 보조 신호)
        double old_width = tr.width_mm;
        tr.width_mm  = clusters[c].width_mm;

        // 폭 급감 감지: 이동 중인 사람의 폭만 검사 (정지 중인 사람의 폭 변화는 무시)
        if (!tr.is_dumped_item && !tr.is_dump_suspect &&
            tr.was_moving && tr.age > 3 &&
            old_width > 0.0 &&
            (old_width - tr.width_mm) > params_.width_drop_threshold_mm)
        {
            tr.width_drop_detected = true;
            tr.width_drop_x_mm     = tr.prev_x_mm;
            tr.width_drop_y_mm     = tr.prev_y_mm;
        } else {
            tr.prev_width_mm = tr.width_mm;
        }

        tr.lost_count = 0;
        tr.age++;
        tr.consecutive_match_count++;
        tr.total_match_count++;

        if (!tr.confirmed &&
            tr.consecutive_match_count >= params_.tentative_confirm_frames)
        {
            tr.confirmed = true;
            if (tr.state == TrackState::Tentative) {
                tr.state = TrackState::Moving;
            }
        }

        // 속도 벡터 계산
        tr.vx_mm = tr.x_mm - tr.prev_x_mm;
        tr.vy_mm = tr.y_mm - tr.prev_y_mm;

        // 칼만 필터 갱신
        // Predict는 Update() 프롤로그(0.5단계)에서 이미 수행됨 → 여기서는 Update만 호출.
        if (!tr.kf.IsInitialized()) {
            tr.kf.Init(tr.x_mm, tr.y_mm,
                       params_.kf_process_noise, params_.kf_measure_noise);
        } else {
            tr.kf.Update(tr.x_mm, tr.y_mm);
        }

        // 궤적 이력 기록 (투기 분리 감지에 사용)
        if (!tr.is_dumped_item && !tr.is_dump_suspect) {
            tr.position_history.push_back({tr.prev_x_mm, tr.prev_y_mm});
            while (tr.position_history.size() > params_.position_history_size) {
                tr.position_history.pop_front();
            }
        }

        double dx = tr.x_mm - tr.prev_x_mm;
        double dy = tr.y_mm - tr.prev_y_mm;
        double move_sq = dx * dx + dy * dy;
        double move_dist = std::sqrt(move_sq);

        tr.cumulative_dist_mm += move_dist;
        tr.recent_move_history.push_back(move_dist);
        while (tr.recent_move_history.size() > 5) {
            tr.recent_move_history.pop_front();
        }

        if (move_sq >= thresh_sq) {
            if (tr.confirmed || tr.is_dump_suspect || tr.is_dumped_item) {
                tr.state        = TrackState::Moving;
            }
            tr.stationary_count = 0;
            tr.stationary_since_frame = 0;
            tr.was_moving       = true;
            tr.departure_fired  = false;
        } else {
            if (tr.stationary_count == 0) {
                tr.stationary_since_frame = frame_count_;
            }
            tr.stationary_count++;

            if (tr.state == TrackState::Moving) {
                tr.state = TrackState::Stationary;
            }

            if (tr.was_moving &&
                !tr.departure_fired &&
                tr.stationary_count >= params_.departure_frame_count)
            {
                tr.state           = TrackState::Departed;
                tr.departure_fired = true;

                DepartureEvent evt;
                evt.track_id          = tr.id;
                evt.timestamp_ms      = NowMs();
                evt.x_mm              = tr.x_mm;
                evt.y_mm              = tr.y_mm;
                evt.stationary_frames = tr.stationary_count;

                dep_events.push_back(evt);

                if (dep_callback_) {
                    dep_callback_(evt);
                }
            }
        }
    }

    // ── 3단계: 미매칭 트랙 → lost 카운트 증가 ──────────────────────
    //   original_track_count 범위만 순회 (이후 추가된 트랙은 새 트랙이라 해당 없음)
    for (size_t t = 0; t < original_track_count; ++t) {
        if (!scratch_track_matched_[t]) {
            tracks_[t].lost_count++;
            tracks_[t].consecutive_match_count = 0;

            // 예측은 0.5단계에서 이미 수행됨 → 여기서는 예측 위치를 트랙 좌표로 복사만.
            if (tracks_[t].kf.IsInitialized()) {
                if (tracks_[t].lost_count <= params_.recovery_max_lost_frames) {
                    tracks_[t].x_mm = tracks_[t].kf.GetX();
                    tracks_[t].y_mm = tracks_[t].kf.GetY();
                }
            }

            if (tracks_[t].is_dump_suspect &&
                tracks_[t].lost_count > params_.suspect_lost_frames_limit)
            {
                tracks_[t].is_dump_suspect      = false;
                tracks_[t].source_id            = -1;
                tracks_[t].suspect_confirm_count = 0;
            }
        }
    }

    UpdatePersonGroups();

    // ── 4단계: 투기 후보(suspect) → 확정(dumped_item) 승격 ──────────
    dd.EvaluateSuspects();

    // ── 5단계: lost_age_limit 초과 트랙 삭제 ────────────────────────
    //   투기 확정 물체(is_dumped_item)는 3배 더 오래 유지
    //   삭제되는 일반 트랙의 위치를 기록하여 오인식 방지
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](const Track& tr) {
                           if (!tr.confirmed &&
                               !tr.is_dump_suspect &&
                               !tr.is_dumped_item &&
                               tr.age + tr.lost_count >= params_.tentative_max_age)
                           {
                               return true;
                           }

                           uint32_t limit = params_.lost_age_limit;
                           if (tr.is_dump_suspect || tr.is_dumped_item) {
                               // 투기 확정/의심 트랙 전용 수명 — TrackerParams 에서 튜닝.
                               limit = params_.dumped_item_lost_age_limit;
                           } else if (tr.was_moving && tr.cumulative_dist_mm >= params_.min_walk_dist_mm) {
                               // 범위 밖으로 나간 사람 ID가 오래 남지 않도록, occlusion 복구 기간까지만 유지한다.
                               limit = params_.recovery_max_lost_frames;
                           }
                           if (tr.lost_count > limit) {
                               // 일반 트랙 삭제 시 위치를 기록 (오인식 방지)
                               if (!tr.is_dumped_item && !tr.is_dump_suspect) {
                                   deleted_positions_.push_back({
                                       tr.x_mm, tr.y_mm,
                                       params_.position_history_size
                                   });
                               }
                               return true;
                           }
                           return false;
                       }),
        tracks_.end()
    );

    // 삭제된 트랙 위치 버퍼 수명 관리
    for (auto& dp : deleted_positions_) {
        if (dp.frames_remaining > 0) dp.frames_remaining--;
    }
    deleted_positions_.erase(
        std::remove_if(deleted_positions_.begin(), deleted_positions_.end(),
                       [](const DeletedTrackPos& dp) {
                           return dp.frames_remaining == 0;
                       }),
        deleted_positions_.end()
    );

    // ── 6단계: 미매칭 클러스터 → 새 트랙 (투기 분리 감지 포함) ─────
    //   scratch_cluster_claimed_ 를 초기화: 매칭된 클러스터는 claimed=true.
    scratch_cluster_claimed_.assign(clusters.size(), false);
    for (size_t c = 0; c < clusters.size(); ++c) {
        if (scratch_cluster_to_track_[c] != -1) scratch_cluster_claimed_[c] = true;
    }

    for (auto& pending : pending_clusters_) {
        if (pending.missed_count < 255) pending.missed_count++;
    }
    for (size_t c = 0; c < clusters.size(); ++c) {
        if (scratch_cluster_claimed_[c]) continue;
        const bool direct_evidence = HasDirectEvidenceForCluster(clusters[c]);
        if (ShouldHoldPendingCluster(clusters[c], direct_evidence)) {
            scratch_cluster_claimed_[c] = true;
        }
    }
    PrunePendingClusters();

    dd.DetectSeparation(clusters);

    // 복구 반경 (잠시 lost된 트랙의 cumulative_dist 등을 계승)
    const double recovery_sq =
        params_.recovery_max_dist_mm * params_.recovery_max_dist_mm;

    for (size_t c = 0; c < clusters.size(); ++c) {
        if (scratch_cluster_claimed_[c]) continue;

        // [정밀도 향상] 크기 필터링: 너무 크거나 너무 작은 노이즈는 트랙 생성 안 함
        if (clusters[c].width_mm < params_.new_track_min_width_mm ||
            clusters[c].width_mm > params_.new_track_max_width_mm) continue;

        Track new_track{};
        new_track.id                 = AllocId();
        new_track.first_seen_frame   = frame_count_;
        new_track.birth_source       = TrackBirthSource::NormalForeground;
        new_track.state              = TrackState::Tentative;
        new_track.x_mm              = clusters[c].centroid_x_mm;
        new_track.y_mm              = clusters[c].centroid_y_mm;
        new_track.prev_x_mm         = new_track.x_mm;
        new_track.prev_y_mm         = new_track.y_mm;
        new_track.width_mm          = clusters[c].width_mm;
        new_track.stationary_count  = 0;
        new_track.lost_count        = 0;
        new_track.age               = 1;
        new_track.was_moving        = false;
        new_track.departure_fired   = false;
        new_track.consecutive_match_count = 1;
        new_track.total_match_count = 1;
        new_track.confirmed         = false;
        new_track.cumulative_dist_mm = 0.0;
        new_track.is_dump_suspect   = false;
        new_track.is_dumped_item    = false;
        new_track.source_id         = -1;
        new_track.dump_alert_fired  = false;
        new_track.suspect_fired     = false;
        new_track.suspect_confirm_count = 0;

        // 다리 오인 방지: 기존 사람 트랙 45cm 이내의 작은(30cm 미만) 새 트랙은
        // age를 0으로 시작하여 투기 주체 인정까지 더 많은 관찰 필요
        for (const auto& tr : tracks_) {
            if (tr.lost_count > 0) continue;
            double dx = new_track.x_mm - tr.x_mm;
            double dy = new_track.y_mm - tr.y_mm;
            double d2 = dx * dx + dy * dy;
            if (d2 < params_.leg_misid_radius_mm * params_.leg_misid_radius_mm &&
                clusters[c].width_mm < params_.leg_misid_max_width_mm) {
                new_track.age = 0;
                break;
            }
        }

        // 트랙 복구: 최근 lost된 트랙 중 가까운 것을 찾아 상태 계승
        // (사람이 몸을 숙이면 LiDAR 프로파일이 급변하여 매칭 실패 → 새 트랙 생성 시
        //  cumulative_dist가 0으로 리셋되는 문제 해결)
        double best_dist_sq = recovery_sq;
        int best_recovery = -1;

        for (size_t t = 0; t < tracks_.size(); ++t) {
            const auto& lt = tracks_[t];
            if (lt.lost_count < 1) continue;
            if (lt.lost_count > params_.recovery_max_lost_frames) continue;
            if (lt.is_dumped_item || lt.is_dump_suspect) continue;

            double rdx = clusters[c].centroid_x_mm - lt.x_mm;
            double rdy = clusters[c].centroid_y_mm - lt.y_mm;
            double rd2 = rdx * rdx + rdy * rdy;
            if (rd2 < best_dist_sq) {
                best_dist_sq = rd2;
                best_recovery = static_cast<int>(t);
            }
        }

        if (best_recovery >= 0) {
            auto& recovered = tracks_[best_recovery];
            new_track.id                 = recovered.id;  // 원본 ID 보존
            new_track.cumulative_dist_mm = recovered.cumulative_dist_mm;
            new_track.was_moving         = recovered.was_moving;
            new_track.age                = recovered.age;
            new_track.position_history   = recovered.position_history;
            new_track.first_seen_frame   = recovered.first_seen_frame;
            new_track.stationary_since_frame = recovered.stationary_since_frame;
            new_track.recent_move_history = recovered.recent_move_history;
            new_track.consecutive_match_count = recovered.confirmed ? 1 : recovered.consecutive_match_count;
            new_track.total_match_count       = recovered.total_match_count + 1;
            new_track.confirmed               = recovered.confirmed;
            new_track.birth_source            = TrackBirthSource::Recovery;
            if (new_track.confirmed) {
                new_track.state = TrackState::Moving;
            }
            recovered.lost_count = params_.lost_age_limit + 1;  // 원본은 제거 예정
        }

        tracks_.push_back(new_track);
    }

    // ── 6.5단계: 기존 정지 트랙 궤적 매칭 재검사 ──────────────────
    dd.TrajectoryMatchStationary();

    UpdatePersonGroups();

    // ── 7단계: 투기물 정지 확인 → 투기 확정 ────────────────────────
    dd.CheckConfirmation(dump_events);

    // 투기 의심(Suspect) 시점 즉시 콜백 (캡처용)
    dd.FireSuspectCallbacks();

    PrunePendingClusters();
}

} // namespace ecowarden
