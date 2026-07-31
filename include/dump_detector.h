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
 * @file  dump_detector.h
 * @brief 투기 감지 전담 모듈 — ClusterTracker 에서 분리 (v6)
 *
 * 4개 하위 기능:
 *   1. SplitDetection   — 1 트랙 → 2 클러스터 분열 감지
 *   2. SeparationDetection — 미매칭 클러스터 ↔ 궤적 이력 근접 감지
 *   3. TrajectoryMatching  — 기존 정지 트랙 궤적 매칭 재검사 (Phase 6.5)
 *   4. ConfirmationGate    — suspect → dumped_item 승격 + 투기 확정
 *
 * ClusterTracker 는 이 클래스에 위임하여 투기 감지를 수행한다.
 */

#pragma once

#include "cluster_tracker.h"
#include "time_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace ecowarden {

enum class DumpSizeBand {
    Tiny,
    Small,
    Medium,
    Large,
    XL,
    Reject,
};

class DumpDetector {
public:
    // 외부 상태 참조 — ClusterTracker가 소유하는 데이터에 대한 비소유 포인터.
    struct Context {
        const TrackerParams* params;
        std::vector<Track>*  tracks;

        // hotspot/deleted 는 DumpDetector 가 읽고 쓴다.
        std::vector<HotspotEntry>* hotspot_positions;
        std::vector<DeletedTrackPos>* deleted_positions;

        // 매칭 결과 (ClusterTracker::AssociateGreedy 의 출력)
        std::vector<int>*  cluster_to_track;
        std::vector<bool>* track_matched;
        std::vector<bool>* cluster_claimed;

        // 새 Track ID 할당
        uint32_t (*alloc_id)(void* user);
        void* alloc_user;
        uint32_t frame_count = 0;

        // 콜백
        DumpingCallback  dump_callback;
        SuspectCallback  suspect_callback;
    };

    explicit DumpDetector(Context ctx) : ctx_(ctx) {}

    // ── Phase 1.5: 클러스터 분열 감지 ───────────────────────────────
    void DetectClusterSplit(const std::vector<Cluster>& clusters) {
        if (!ctx_.params->enable_dumping_detection) return;

        for (auto& tr : *ctx_.tracks) { tr.split_detected = false; }

        for (size_t t = 0; t < ctx_.tracks->size(); ++t) {
            if (!(*ctx_.track_matched)[t]) continue;
            auto& tr = (*ctx_.tracks)[t];
            if (tr.is_dumped_item || tr.is_dump_suspect) continue;
            if (tr.cumulative_dist_mm < ctx_.params->min_walk_dist_mm) continue;

            double split_radius = std::min(
                tr.width_mm + ctx_.params->split_radius_margin_mm,
                ctx_.params->split_radius_max_mm);
            double split_radius_sq = split_radius * split_radius;

            for (size_t c = 0; c < clusters.size(); ++c) {
                if ((*ctx_.cluster_to_track)[c] != -1) continue;
                if (clusters[c].points.size() < ctx_.params->min_dump_candidate_points) continue;
                if (clusters[c].width_mm < ctx_.params->min_dump_candidate_width_mm) continue;

                double dx = clusters[c].centroid_x_mm - tr.x_mm;
                double dy = clusters[c].centroid_y_mm - tr.y_mm;
                if (dx * dx + dy * dy < split_radius_sq) {
                    tr.split_detected = true;
                    tr.split_x_mm = clusters[c].centroid_x_mm;
                    tr.split_y_mm = clusters[c].centroid_y_mm;
                    break;
                }
            }
        }
    }

    // ── Phase 4: suspect → dumped_item 승격 ─────────────────────────
    void EvaluateSuspects() {
        if (!ctx_.params->enable_dumping_detection) return;

        const double leg_prox_sq =
            ctx_.params->leg_proximity_radius_mm * ctx_.params->leg_proximity_radius_mm;

        for (auto& tr : *ctx_.tracks) {
            if (!tr.is_dump_suspect) continue;
            if (tr.is_dumped_item)   continue;

            bool near_other_person = false;
            uint32_t source_group_id = 0;
            for (const auto& source : *ctx_.tracks) {
                if (static_cast<int>(source.id) == tr.source_id) {
                    source_group_id = source.person_group_id;
                    break;
                }
            }
            for (const auto& other : *ctx_.tracks) {
                if (other.id == tr.id) continue;
                if (static_cast<int>(other.id) == tr.source_id) continue;
                if (other.is_dumped_item || other.is_dump_suspect) continue;
                if (source_group_id != 0 && other.person_group_id == source_group_id) continue;
                if (other.is_object_candidate) continue;

                double dx = tr.x_mm - other.x_mm;
                double dy = tr.y_mm - other.y_mm;
                if (dx * dx + dy * dy < leg_prox_sq) {
                    near_other_person = true;
                    break;
                }
            }

            if (near_other_person) {
                DebugBlock("near_unrelated_person", tr);
                if (!tr.source_locked) {
                    tr.is_dump_suspect = false;
                    tr.source_id = -1;
                    tr.suspect_confirm_count = 0;
                }
                continue;
            }

            tr.suspect_confirm_count++;
            tr.suspect_width_sum += tr.width_mm;
            tr.suspect_width_sq_sum += tr.width_mm * tr.width_mm;

            if (tr.suspect_confirm_count >= 3) {
                double mean = tr.suspect_width_sum / tr.suspect_confirm_count;
                double variance = (tr.suspect_width_sq_sum / tr.suspect_confirm_count) - (mean * mean);
                if (variance > 2500.0) {
                    DebugBlock("width_variance", tr);
                    tr.is_dump_suspect = false;
                    tr.source_id = -1;
                    tr.suspect_confirm_count = 0;
                    tr.suspect_width_sum = 0.0;
                    tr.suspect_width_sq_sum = 0.0;
                    continue;
                }
            }

            uint32_t required_frames = ctx_.params->separation_confirm_frames;
            const DumpSizeBand size_band = GetDumpSizeBand(tr.width_mm);
            if (size_band == DumpSizeBand::Reject) {
                DebugBlock("size_band_reject", tr);
                tr.is_dump_suspect = false;
                tr.source_id = -1;
                tr.suspect_confirm_count = 0;
                continue;
            }
            required_frames = std::max(required_frames,
                                       RequiredSuspectFrames(size_band));
            if (IsInHotspot(tr.x_mm, tr.y_mm) &&
                required_frames > ctx_.params->hotspot_boost_frames)
            {
                required_frames -= ctx_.params->hotspot_boost_frames;
                if (required_frames < ctx_.params->hotspot_boost_min_frames) {
                    required_frames = ctx_.params->hotspot_boost_min_frames;
                }
            }

            if (tr.suspect_confirm_count >= required_frames) {
                const bool ownership_gate =
                    tr.source_locked &&
                    tr.newly_created_after_source;
                const bool source_gate =
                    ownership_gate &&
                    tr.direct_dump_evidence;
                // 봉투는 centroid가 흔들리므로 5mm/frame 고정 게이트 대신
                // 운영 파라미터로 완화된 정지 판정을 사용한다.
                bool object_stationary =
                    IsObjectStable(tr, ctx_.params->object_stable_avg_speed_mm) ||
                    (std::fabs(tr.vx_mm) < ctx_.params->dumped_item_still_speed_mm &&
                    std::fabs(tr.vy_mm) < ctx_.params->dumped_item_still_speed_mm);
                if (!object_stationary) {
                    DebugBlock("not_stable", tr);
                    continue;
                }
                if (tr.total_match_count <
                    ctx_.params->dump_confirm_min_total_matches) {
                    DebugBlock("not_enough_observations", tr);
                    continue;
                }

                bool person_departing = false;
                bool person_found = false;
                for (const auto& p : *ctx_.tracks) {
                    if (static_cast<int>(p.id) == tr.source_id &&
                        p.lost_count == 0) {
                        person_found = true;
                        const double px = p.person_group_id ? p.person_group_x_mm : p.x_mm;
                        const double py = p.person_group_id ? p.person_group_y_mm : p.y_mm;
                        double dx = px - tr.x_mm;
                        double dy = py - tr.y_mm;
                        double dist = std::sqrt(dx * dx + dy * dy);

                        if (tr.source_last_dist_mm > 0.0) {
                            if (dist > tr.source_last_dist_mm + 5.0) {
                                tr.source_dist_increase_count++;
                            } else if (dist + 15.0 < tr.source_last_dist_mm) {
                                tr.source_dist_increase_count = 0;
                            }
                        }
                        tr.source_last_dist_mm = dist;

                        double nx = dx / std::max(dist, 1.0);
                        double ny = dy / std::max(dist, 1.0);
                        double recede_speed = p.vx_mm * nx + p.vy_mm * ny;
                        const bool fast_trend_ok =
                            tr.source_dist_increase_count >= ctx_.params->fast_confirm_trend_frames;
                        const bool trend_ok =
                            tr.source_dist_increase_count >= ctx_.params->person_depart_trend_frames;
                        const bool strong_recede_ok =
                            trend_ok ||
                            recede_speed > ctx_.params->receding_velocity_threshold;
                        const bool fast_confirm =
                            source_gate &&
                            tr.suspect_confirm_count >= ctx_.params->fast_confirm_min_frames &&
                            dist >= ctx_.params->fast_confirm_depart_mm &&
                            fast_trend_ok;
                        const bool trajectory_birth_confirm =
                            ownership_gate &&
                            !tr.direct_dump_evidence &&
                            tr.suspect_confirm_count >= ctx_.params->fast_confirm_min_frames &&
                            object_stationary &&
                            dist >= ctx_.params->person_depart_dist_mm &&
                            !HasUnrelatedPersonNear(tr,
                                ctx_.params->near_unrelated_person_hold_mm);

                        if (source_gate &&
                            dist >= ctx_.params->strong_person_depart_dist_mm &&
                            strong_recede_ok) {
                            person_departing = true;
                        } else if (fast_confirm) {
                            person_departing = true;
                        } else if (trajectory_birth_confirm) {
                            person_departing = true;
                        } else if (source_gate &&
                                   dist >= ctx_.params->person_depart_dist_mm &&
                                   (trend_ok || recede_speed > 0.0)) {
                            person_departing = true;
                        } else if (source_gate &&
                                   dist >= ctx_.params->soft_person_depart_dist_mm &&
                                   trend_ok &&
                                   recede_speed > 0.0) {
                            person_departing = true;
                        }
                        break;
                    }
                }

                // 투기 주체를 찾을 수 없으면 시야 밖 이탈로 본다. 단, 너무 짧은
                // suspect는 오탐 위험이 있으므로 추세 프레임만큼 관찰한 뒤 승격한다.
                if (!person_found &&
                    ownership_gate &&
                    tr.suspect_confirm_count >= ctx_.params->source_lost_confirm_frames) {
                    person_departing = true;
                } else if (!person_found && !person_departing) {
                    double dx = tr.source_x_mm - tr.x_mm;
                    double dy = tr.source_y_mm - tr.y_mm;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (ownership_gate &&
                        dist >= ctx_.params->strong_person_depart_dist_mm) {
                        person_departing = true;
                    }
                }

                if (person_departing) {
                    DebugConfirmCandidate("DUMP-CONFIRM", tr);
                    tr.is_dump_suspect = false;
                    tr.is_dumped_item  = true;
                } else {
                    DebugBlock("source_not_departing", tr);
                }
            }
        }
    }

    // ── Phase 6 내부: 미매칭 클러스터에서 투기 분리 감지 ────────────
    void DetectSeparation(const std::vector<Cluster>& clusters) {
        if (!ctx_.params->enable_dumping_detection) return;

        const double sep_prev_sq =
            ctx_.params->separation_max_dist_mm * ctx_.params->separation_max_dist_mm;
        const double sep_cur_sq =
            ctx_.params->separation_min_dist_from_current_mm *
            ctx_.params->separation_min_dist_from_current_mm;
        const double recovery_sq =
            ctx_.params->recovery_max_dist_mm * ctx_.params->recovery_max_dist_mm;

        for (size_t c = 0; c < clusters.size(); ++c) {
            if ((*ctx_.cluster_to_track)[c] != -1) continue;
            if ((*ctx_.cluster_claimed)[c])        continue;

            if (clusters[c].width_mm < ctx_.params->min_dump_candidate_width_mm) {
                DebugClusterBlock("size_band_reject", clusters[c]);
                continue;
            }
            if (clusters[c].points.size() < ctx_.params->min_dump_candidate_points) {
                DebugClusterBlock("points_reject", clusters[c]);
                continue;
            }
            if (clusters[c].width_mm > ctx_.params->dump_candidate_max_width_mm) {
                DebugClusterBlock("size_band_reject", clusters[c]);
                continue;
            }
            const DumpSizeBand cluster_band = GetDumpSizeBand(clusters[c].width_mm);
            if (cluster_band == DumpSizeBand::Reject) {
                DebugClusterBlock("size_band_reject", clusters[c]);
                continue;
            }
            if ((cluster_band == DumpSizeBand::Large &&
                 clusters[c].points.size() < 4) ||
                (cluster_band == DumpSizeBand::XL &&
                 clusters[c].points.size() < 5)) {
                DebugClusterBlock("points_reject", clusters[c]);
                continue;
            }

            // lost track recovery check
            bool matches_lost_track = false;
            for (const auto& lt : *ctx_.tracks) {
                if (lt.lost_count < 1) continue;
                if (lt.is_dumped_item || lt.is_dump_suspect) continue;
                double rdx = clusters[c].centroid_x_mm - lt.x_mm;
                double rdy = clusters[c].centroid_y_mm - lt.y_mm;
                if (rdx * rdx + rdy * rdy < recovery_sq) { matches_lost_track = true; break; }
            }
            if (!matches_lost_track) {
                for (const auto& dp : *ctx_.deleted_positions) {
                    double rdx = clusters[c].centroid_x_mm - dp.x_mm;
                    double rdy = clusters[c].centroid_y_mm - dp.y_mm;
                    if (rdx * rdx + rdy * rdy < recovery_sq) { matches_lost_track = true; break; }
                }
            }
            if (matches_lost_track) {
                DebugClusterBlock("lost_track_recovery", clusters[c]);
                continue;
            }

            if (IsInHotspot(clusters[c].centroid_x_mm, clusters[c].centroid_y_mm)) {
                DebugClusterBlock("hotspot", clusters[c]);
                continue;
            }

            // active dump block
            bool near_dumped = false;
            const double dump_block_sq =
                ctx_.params->active_dump_block_radius_mm * ctx_.params->active_dump_block_radius_mm;
            for (const auto& dt : *ctx_.tracks) {
                if (!dt.is_dumped_item) continue;
                double ddx = clusters[c].centroid_x_mm - dt.x_mm;
                double ddy = clusters[c].centroid_y_mm - dt.y_mm;
                if (ddx * ddx + ddy * ddy < dump_block_sq) { near_dumped = true; break; }
            }
            if (near_dumped) {
                DebugClusterBlock("near_existing_dump", clusters[c]);
                continue;
            }

            // person proximity hold
            bool near_existing_person = false;
            const double person_hold_sq =
                ctx_.params->person_proximity_hold_radius_mm * ctx_.params->person_proximity_hold_radius_mm;
            for (const auto& tr : *ctx_.tracks) {
                if (tr.is_dumped_item || tr.is_dump_suspect) continue;
                if (tr.is_object_candidate) continue;
                double dx = clusters[c].centroid_x_mm - tr.x_mm;
                double dy = clusters[c].centroid_y_mm - tr.y_mm;
                if (dx * dx + dy * dy < person_hold_sq) { near_existing_person = true; break; }
            }
            if (near_existing_person) {
                DebugClusterBlock("near_existing_person", clusters[c]);
                (*ctx_.cluster_claimed)[c] = true;
                continue;
            }

            // trajectory matching
            bool is_separation = false;
            int  person_id     = -1;
            const double cx = clusters[c].centroid_x_mm;
            const double cy = clusters[c].centroid_y_mm;

            for (const auto& tr : *ctx_.tracks) {
                if (tr.is_dumped_item || tr.is_dump_suspect) continue;
                if (tr.is_object_candidate) continue;
                if (tr.lost_count > 1) continue;
                if (tr.cumulative_dist_mm < ctx_.params->min_walk_dist_mm) continue;

                if (tr.age >= ctx_.params->min_age_for_dump) {
                    double dx_cur = cx - tr.x_mm;
                    double dy_cur = cy - tr.y_mm;
                    double dist_cur_sq = dx_cur * dx_cur + dy_cur * dy_cur;

                    if (dist_cur_sq >= sep_cur_sq) {
                        double dx_prev = cx - tr.prev_x_mm;
                        double dy_prev = cy - tr.prev_y_mm;
                        if (dx_prev * dx_prev + dy_prev * dy_prev < sep_prev_sq) {
                            is_separation = true; person_id = static_cast<int>(tr.id); break;
                        }
                        for (const auto& pos : tr.position_history) {
                            double dhx = cx - pos.first;
                            double dhy = cy - pos.second;
                            if (dhx * dhx + dhy * dhy < sep_prev_sq) {
                                is_separation = true; person_id = static_cast<int>(tr.id); break;
                            }
                        }
                        if (is_separation) break;
                    }
                }

                if (tr.width_drop_detected) {
                    const double width_drop_match_sq =
                        ctx_.params->split_match_radius_mm *
                        ctx_.params->split_match_radius_mm;
                    double dwx = cx - tr.width_drop_x_mm;
                    double dwy = cy - tr.width_drop_y_mm;
                    if (dwx * dwx + dwy * dwy < width_drop_match_sq) {
                        is_separation = true; person_id = static_cast<int>(tr.id); break;
                    }
                }

                if (tr.split_detected) {
                    const double split_match_sq =
                        ctx_.params->split_match_radius_mm * ctx_.params->split_match_radius_mm;
                    double dsx = cx - tr.split_x_mm;
                    double dsy = cy - tr.split_y_mm;
                    if (dsx * dsx + dsy * dsy < split_match_sq) {
                        is_separation = true; person_id = static_cast<int>(tr.id); break;
                    }
                }
            }

            if (!is_separation) {
                DebugClusterBlock("no_direct_evidence", clusters[c]);
                continue;
            }

            Track new_track{};
            new_track.id             = ctx_.alloc_id(ctx_.alloc_user);
            new_track.first_seen_frame = ctx_.frame_count;
            new_track.stationary_since_frame = ctx_.frame_count;
            new_track.state          = TrackState::Stationary;
            new_track.x_mm           = clusters[c].centroid_x_mm;
            new_track.y_mm           = clusters[c].centroid_y_mm;
            new_track.prev_x_mm     = new_track.x_mm;
            new_track.prev_y_mm     = new_track.y_mm;
            new_track.width_mm      = clusters[c].width_mm;
            new_track.stationary_count = 0;
            new_track.lost_count    = 0;
            new_track.age           = 1;
            new_track.was_moving    = false;
            new_track.departure_fired = false;
            new_track.consecutive_match_count = 1;
            new_track.total_match_count = 1;
            new_track.confirmed = true;
            new_track.cumulative_dist_mm = 0.0;
            new_track.is_dump_suspect = true;
            new_track.is_dumped_item  = false;
            new_track.source_id     = person_id;
            new_track.source_locked = true;
            new_track.source_bind_frame = ctx_.frame_count;
            new_track.newly_created_after_source = true;
            new_track.direct_dump_evidence = true;
            new_track.birth_source = TrackBirthSource::SeparationDirect;
            new_track.dump_alert_fired = false;
            new_track.suspect_confirm_count = 0;

            for (const auto& tr2 : *ctx_.tracks) {
                if (static_cast<int>(tr2.id) == person_id) {
                    new_track.source_person_entity_id =
                        tr2.person_group_id ? tr2.person_group_id : tr2.id;
                    new_track.source_x_mm =
                        tr2.person_group_id ? tr2.person_group_x_mm : tr2.x_mm;
                    new_track.source_y_mm =
                        tr2.person_group_id ? tr2.person_group_y_mm : tr2.y_mm;
                    new_track.source_cumulative_dist_mm = tr2.cumulative_dist_mm;
                    break;
                }
            }

            (*ctx_.cluster_claimed)[c] = true;
            ctx_.tracks->push_back(new_track);
            DebugSuspect("DUMP-SUSPECT", new_track);
        }
    }

    // ── Phase 6.5: 기존 정지 트랙 궤적 매칭 재검사 ─────────────────
    void TrajectoryMatchStationary() {
        if (!ctx_.params->enable_dumping_detection) return;

        const double sep_prev_sq =
            ctx_.params->separation_max_dist_mm * ctx_.params->separation_max_dist_mm;
        const double dump_block_sq =
            ctx_.params->active_dump_block_radius_mm * ctx_.params->active_dump_block_radius_mm;

        for (auto& obj : *ctx_.tracks) {
            if (obj.is_dump_suspect || obj.is_dumped_item) continue;
            if (obj.dump_alert_fired) continue;
            if (obj.state == TrackState::Moving) continue;
            if (obj.width_mm > ctx_.params->dump_candidate_max_width_mm ||
                obj.width_mm < ctx_.params->min_dump_candidate_width_mm) {
                DebugBlock("size_band_reject", obj);
                continue;
            }
            const DumpSizeBand obj_band = GetDumpSizeBand(obj.width_mm);
            if (obj_band == DumpSizeBand::Reject) {
                DebugBlock("size_band_reject", obj);
                continue;
            }
            if (obj.stationary_count < 5) {
                DebugBlock("not_stable", obj);
                continue;
            }
            if (obj.age > ctx_.params->dump_candidate_birth_frames) {
                DebugBlock("old_stationary_object", obj);
                continue;
            }
            if (obj.cumulative_dist_mm > ctx_.params->stationary_track_max_cum_dist_mm) {
                DebugBlock("moving_object", obj);
                continue;
            }

            if (IsInHotspot(obj.x_mm, obj.y_mm)) {
                DebugBlock("hotspot", obj);
                continue;
            }

            bool near_dumped = false;
            for (const auto& dt : *ctx_.tracks) {
                if (!dt.is_dumped_item) continue;
                double ddx = obj.x_mm - dt.x_mm;
                double ddy = obj.y_mm - dt.y_mm;
                if (ddx * ddx + ddy * ddy < dump_block_sq) { near_dumped = true; break; }
            }
            if (near_dumped) {
                DebugBlock("near_existing_dump", obj);
                continue;
            }

            for (const auto& person : *ctx_.tracks) {
                if (person.id == obj.id) continue;
                if (person.is_dumped_item || person.is_dump_suspect) continue;
                if (person.is_object_candidate) continue;
                if (person.lost_count > ctx_.params->recovery_max_lost_frames) continue;
                if (person.cumulative_dist_mm < ctx_.params->min_walk_dist_mm) continue;
                if (person.age < ctx_.params->min_age_for_dump) continue;

                double dx_cur = obj.x_mm - person.x_mm;
                double dy_cur = obj.y_mm - person.y_mm;
                if (dx_cur * dx_cur + dy_cur * dy_cur < 200.0 * 200.0) continue;

                bool on_trajectory = false;
                for (const auto& pos : person.position_history) {
                    double dhx = obj.x_mm - pos.first;
                    double dhy = obj.y_mm - pos.second;
                    if (dhx * dhx + dhy * dhy < sep_prev_sq) { on_trajectory = true; break; }
                }

                if (on_trajectory) {
                    const double aux_sq =
                        ctx_.params->split_match_radius_mm *
                        ctx_.params->split_match_radius_mm;

                    bool width_drop_near = false;
                    if (person.width_drop_detected) {
                        const double wdx = obj.x_mm - person.width_drop_x_mm;
                        const double wdy = obj.y_mm - person.width_drop_y_mm;
                        width_drop_near = (wdx * wdx + wdy * wdy) <= aux_sq;
                    }

                    bool split_near = false;
                    if (person.split_detected) {
                        const double sdx = obj.x_mm - person.split_x_mm;
                        const double sdy = obj.y_mm - person.split_y_mm;
                        split_near = (sdx * sdx + sdy * sdy) <= aux_sq;
                    }

                    const bool has_secondary_signal =
                        width_drop_near || split_near;
                    const double current_dist = std::sqrt(dx_cur * dx_cur + dy_cur * dy_cur);
                    const bool birth_trajectory_signal =
                        obj.age <= ctx_.params->dump_candidate_birth_frames &&
                        obj.stationary_count >=
                            ctx_.params->trajectory_birth_suspect_min_stationary_frames &&
                        obj.total_match_count >=
                            ctx_.params->trajectory_birth_min_total_matches &&
                        obj.width_mm <=
                            ctx_.params->trajectory_birth_suspect_max_width_mm &&
                        obj.cumulative_dist_mm <=
                            ctx_.params->stationary_track_max_cum_dist_mm &&
                        current_dist >= ctx_.params->trajectory_birth_depart_mm &&
                        !HasUnrelatedPersonNear(obj,
                            ctx_.params->near_unrelated_person_hold_mm);

                    if (!has_secondary_signal && !birth_trajectory_signal) {
                        DebugBlock("no_direct_evidence", obj);
                        continue;
                    }

                    obj.is_dump_suspect = true;
                    obj.source_id = static_cast<int>(person.id);
                    obj.source_locked = true;
                    obj.source_bind_frame = ctx_.frame_count;
                    obj.source_person_entity_id =
                        person.person_group_id ? person.person_group_id : person.id;
                    obj.source_x_mm =
                        person.person_group_id ? person.person_group_x_mm : person.x_mm;
                    obj.source_y_mm =
                        person.person_group_id ? person.person_group_y_mm : person.y_mm;
                    obj.source_cumulative_dist_mm = person.cumulative_dist_mm;
                    obj.newly_created_after_source = true;
                    obj.direct_dump_evidence = has_secondary_signal;
                    obj.suspect_confirm_count = 0;
                    obj.suspect_width_sum = 0.0;
                    obj.suspect_width_sq_sum = 0.0;
                    DebugSuspect("DUMP-SUSPECT", obj);
                    break;
                }
            }
        }
    }

    // ── Phase 7: 투기물 정지 확인 → 투기 확정 ──────────────────────
    void CheckConfirmation(std::vector<DumpingEvent>& dump_events) {
        if (!ctx_.params->enable_dumping_detection) return;

        for (auto& tr : *ctx_.tracks) {
            if (!tr.is_dumped_item)     continue;
            if (tr.dump_alert_fired)    continue;
            const bool object_stable =
                IsObjectStable(tr, ctx_.params->object_stable_avg_speed_mm);
            if (!object_stable &&
                tr.stationary_count < ctx_.params->fast_confirm_min_frames) continue;
            if (tr.total_match_count <
                ctx_.params->dump_confirm_min_total_matches) {
                DebugBlock("not_enough_observations", tr);
                continue;
            }

            bool person_found = false;
            double p_x = 0.0, p_y = 0.0, p_cum = 0.0;
            double p_vx = 0.0, p_vy = 0.0;

            for (const auto& p : *ctx_.tracks) {
                if (static_cast<int>(p.id) == tr.source_id &&
                    p.lost_count == 0) {
                    p_x = p.person_group_id ? p.person_group_x_mm : p.x_mm;
                    p_y = p.person_group_id ? p.person_group_y_mm : p.y_mm;
                    p_cum = p.cumulative_dist_mm;
                    p_vx = p.vx_mm;
                    p_vy = p.vy_mm;
                    person_found = true; break;
                }
            }

            if (!person_found) {
                p_x = tr.source_x_mm; p_y = tr.source_y_mm;
                p_cum = tr.source_cumulative_dist_mm;
            }

            double dx = p_x - tr.x_mm;
            double dy = p_y - tr.y_mm;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (tr.source_last_dist_mm > 0.0) {
                if (dist > tr.source_last_dist_mm + 5.0) {
                    tr.source_dist_increase_count++;
                } else if (dist + 15.0 < tr.source_last_dist_mm) {
                    tr.source_dist_increase_count = 0;
                }
            }
            tr.source_last_dist_mm = dist;

            const bool trend_ok =
                tr.source_dist_increase_count >= ctx_.params->person_depart_trend_frames;
            const bool fast_trend_ok =
                tr.source_dist_increase_count >= ctx_.params->fast_confirm_trend_frames;
            const double nx = dx / std::max(dist, 1.0);
            const double ny = dy / std::max(dist, 1.0);
            const double recede_speed = p_vx * nx + p_vy * ny;
            const bool strong_recede_ok =
                trend_ok ||
                recede_speed > ctx_.params->receding_velocity_threshold;
            const bool hard_depart =
                tr.source_locked &&
                tr.newly_created_after_source &&
                tr.direct_dump_evidence &&
                dist >= ctx_.params->strong_person_depart_dist_mm &&
                tr.stationary_count >= ctx_.params->fast_confirm_min_frames &&
                strong_recede_ok;
            const bool fast_depart =
                tr.source_locked &&
                tr.newly_created_after_source &&
                tr.direct_dump_evidence &&
                object_stable &&
                tr.stationary_count >= ctx_.params->fast_confirm_min_frames &&
                dist >= ctx_.params->fast_confirm_depart_mm &&
                fast_trend_ok &&
                !HasUnrelatedPersonNear(tr, ctx_.params->near_unrelated_person_hold_mm);
            const bool default_depart =
                tr.source_locked &&
                tr.newly_created_after_source &&
                dist >= ctx_.params->person_depart_dist_mm &&
                trend_ok &&
                (object_stable || tr.stationary_count >= 8);
            const bool trajectory_birth_depart =
                tr.source_locked &&
                tr.newly_created_after_source &&
                !tr.direct_dump_evidence &&
                object_stable &&
                tr.stationary_count >= ctx_.params->fast_confirm_min_frames &&
                dist >= ctx_.params->person_depart_dist_mm &&
                !HasUnrelatedPersonNear(tr, ctx_.params->near_unrelated_person_hold_mm);
            const bool source_lost_depart =
                !person_found &&
                tr.source_locked &&
                tr.newly_created_after_source &&
                tr.stationary_count >= ctx_.params->source_lost_confirm_frames;
            if (!fast_depart && !hard_depart && !default_depart &&
                !trajectory_birth_depart && !source_lost_depart) {
                DebugBlock("source_not_departing", tr);
                continue;
            }

            if (person_found) {
                double dot = (p_x - tr.x_mm) * (p_x - tr.prev_x_mm) +
                             (p_y - tr.y_mm) * (p_y - tr.prev_y_mm);
                if (dot < 0 && !fast_depart && !hard_depart && !trend_ok) {
                    DebugBlock("source_not_departing", tr);
                    continue;
                }
            }

            DumpingEvent evt;
            evt.person_track_id = tr.source_person_entity_id != 0
                ? tr.source_person_entity_id
                : static_cast<uint32_t>(tr.source_id);
            evt.person_x_mm = p_x;
            evt.person_y_mm = p_y;
            evt.person_cumulative_dist_mm = p_cum;
            evt.object_track_id = tr.id;
            evt.object_x_mm = tr.x_mm;
            evt.object_y_mm = tr.y_mm;
            evt.timestamp_ms = NowMs();

            dump_events.push_back(evt);
            if (ctx_.dump_callback) ctx_.dump_callback(evt);
            tr.dump_alert_fired = true;
            DebugConfirmCandidate("DUMP-CONFIRM", tr);

            // Hotspot 등록
            if (ctx_.hotspot_positions->size() >= ctx_.params->max_hotspots) {
                auto oldest = std::min_element(
                    ctx_.hotspot_positions->begin(), ctx_.hotspot_positions->end(),
                    [](const auto& a, const auto& b) { return a.frames_remaining < b.frames_remaining; });
                *oldest = ctx_.hotspot_positions->back();
                ctx_.hotspot_positions->pop_back();
            }
            ctx_.hotspot_positions->push_back({tr.x_mm, tr.y_mm, ctx_.params->hotspot_ttl_frames});

            std::printf("\n[ALERT] 쓰레기 투기 최종 확정!\n"
                        " -> 투기 주체 ID : %d\n"
                        " -> 투기물 ID    : %u\n"
                        " -> 투기물 위치  : (%.0f, %.0f) mm\n\n",
                        tr.source_id, tr.id, tr.x_mm, tr.y_mm);
        }
    }

    // ── Suspect 콜백 발화 ──────────────────────────────────────────
    void FireSuspectCallbacks() {
        if (!ctx_.suspect_callback) return;
        for (auto& tr : *ctx_.tracks) {
            if (tr.is_dump_suspect && !tr.suspect_fired) {
                if (!tr.source_locked ||
                    !tr.newly_created_after_source ||
                    !tr.direct_dump_evidence) {
                    DebugBlock("no_direct_evidence", tr);
                    continue;
                }
                if (tr.suspect_confirm_count <
                    ctx_.params->min_suspect_callback_frames) {
                    DebugBlock("visible_callback_wait", tr);
                    continue;
                }
                if (!IsObjectStable(tr, ctx_.params->suspect_callback_max_speed_mm)) {
                    DebugBlock("not_stable", tr);
                    continue;
                }
                if (HasUnrelatedPersonNear(tr, ctx_.params->near_unrelated_person_hold_mm)) {
                    DebugBlock("near_unrelated_person", tr);
                    continue;
                }
                if (tr.suspect_confirm_count >= 2) {
                    const double mean =
                        tr.suspect_width_sum / tr.suspect_confirm_count;
                    const double variance =
                        (tr.suspect_width_sq_sum / tr.suspect_confirm_count) -
                        (mean * mean);
                    if (variance > ctx_.params->suspect_callback_max_width_variance) {
                        DebugBlock("width_variance", tr);
                        continue;
                    }
                }
                DumpingSuspectEvent sevt;
                sevt.person_track_id = tr.source_person_entity_id != 0
                    ? tr.source_person_entity_id
                    : static_cast<uint32_t>(tr.source_id);
                sevt.object_track_id = tr.id;
                sevt.object_x_mm = tr.x_mm;
                sevt.object_y_mm = tr.y_mm;
                sevt.timestamp_ms = NowMs();
                ctx_.suspect_callback(sevt);
                tr.suspect_fired = true;
            }
        }
    }

private:
    Context ctx_;

    bool DebugEnabled() const {
        const char* level = std::getenv("ECOWARDEN_LOG_LEVEL");
        return level &&
               (std::strcmp(level, "debug") == 0 ||
                std::strcmp(level, "frame") == 0 ||
                std::strcmp(level, "dump") == 0);
    }

    const char* BandName(DumpSizeBand band) const {
        switch (band) {
            case DumpSizeBand::Tiny: return "Tiny";
            case DumpSizeBand::Small: return "Small";
            case DumpSizeBand::Medium: return "Medium";
            case DumpSizeBand::Large: return "Large";
            case DumpSizeBand::XL: return "XL";
            case DumpSizeBand::Reject: return "Reject";
        }
        return "Unknown";
    }

    void DebugBlock(const char* reason, const Track& tr) const {
        if (!DebugEnabled()) return;
        std::printf("[DUMP-BLOCK] reason=%s frame=%u object_id=%u source_id=%d "
                    "x=%.0f y=%.0f width=%.0f band=%s stc=%u age=%u\n",
                    reason, ctx_.frame_count, tr.id, tr.source_id,
                    tr.x_mm, tr.y_mm, tr.width_mm,
                    BandName(GetDumpSizeBand(tr.width_mm)),
                    tr.stationary_count, tr.age);
    }

    void DebugClusterBlock(const char* reason, const Cluster& c) const {
        if (!DebugEnabled()) return;
        std::printf("[DUMP-BLOCK] reason=%s frame=%u cluster=(%.0f,%.0f) "
                    "width=%.0f points=%zu band=%s\n",
                    reason, ctx_.frame_count, c.centroid_x_mm, c.centroid_y_mm,
                    c.width_mm, c.points.size(),
                    BandName(GetDumpSizeBand(c.width_mm)));
    }

    void DebugSuspect(const char* label, const Track& tr) const {
        if (!DebugEnabled()) return;
        std::printf("[%s] frame=%u object_id=%u source_id=%d "
                    "source_entity=%u x=%.0f y=%.0f width=%.0f band=%s\n",
                    label, ctx_.frame_count, tr.id, tr.source_id,
                    tr.source_person_entity_id, tr.x_mm, tr.y_mm, tr.width_mm,
                    BandName(GetDumpSizeBand(tr.width_mm)));
    }

    void DebugConfirmCandidate(const char* label, const Track& tr) const {
        if (!DebugEnabled()) return;
        std::printf("[%s] frame=%u object_id=%u source_id=%d "
                    "stable=%u dist_increase=%u stc=%u\n",
                    label, ctx_.frame_count, tr.id, tr.source_id,
                    IsObjectStable(tr, ctx_.params->object_stable_avg_speed_mm) ? 1u : 0u,
                    tr.source_dist_increase_count, tr.stationary_count);
    }

    DumpSizeBand GetDumpSizeBand(double width_mm) const {
        if (width_mm < ctx_.params->min_dump_candidate_width_mm ||
            width_mm > ctx_.params->dump_candidate_max_width_mm) {
            return DumpSizeBand::Reject;
        }
        if (width_mm < 100.0) return DumpSizeBand::Tiny;
        if (width_mm < 180.0) return DumpSizeBand::Small;
        if (width_mm < 350.0) return DumpSizeBand::Medium;
        if (width_mm < 650.0) return DumpSizeBand::Large;
        return DumpSizeBand::XL;
    }

    uint32_t RequiredSuspectFrames(DumpSizeBand band) const {
        switch (band) {
            case DumpSizeBand::Tiny:   return ctx_.params->dump_frames_tiny;
            case DumpSizeBand::Small:  return ctx_.params->dump_frames_small;
            case DumpSizeBand::Medium: return ctx_.params->dump_frames_medium;
            case DumpSizeBand::Large:  return ctx_.params->dump_frames_large;
            case DumpSizeBand::XL:     return ctx_.params->dump_frames_xl;
            default:                   return ctx_.params->dump_frames_small;
        }
    }

    bool IsObjectStable(const Track& tr, double max_avg_speed_mm) const {
        if (tr.recent_move_history.empty()) {
            return tr.stationary_count >= ctx_.params->fast_confirm_min_frames;
        }
        double sum = 0.0;
        for (double d : tr.recent_move_history) sum += d;
        const double avg = sum / static_cast<double>(tr.recent_move_history.size());
        return avg <= max_avg_speed_mm ||
               tr.stationary_count >= ctx_.params->fast_confirm_min_frames;
    }

    bool HasUnrelatedPersonNear(const Track& obj, double radius_mm) const {
        const double radius_sq = radius_mm * radius_mm;
        for (const auto& other : *ctx_.tracks) {
            if (other.id == obj.id) continue;
            if (static_cast<int>(other.id) == obj.source_id) continue;
            if (other.is_dumped_item || other.is_dump_suspect) continue;
            if (other.is_object_candidate) continue;
            if (obj.source_person_entity_id != 0 &&
                other.person_group_id == obj.source_person_entity_id) {
                continue;
            }
            double dx = obj.x_mm - other.x_mm;
            double dy = obj.y_mm - other.y_mm;
            if (dx * dx + dy * dy <= radius_sq) return true;
        }
        return false;
    }

    bool IsInHotspot(double x, double y) const {
        const double r_sq = ctx_.params->hotspot_radius_mm * ctx_.params->hotspot_radius_mm;
        for (const auto& hp : *ctx_.hotspot_positions) {
            double dx = x - hp.x_mm;
            double dy = y - hp.y_mm;
            if (dx * dx + dy * dy < r_sq) return true;
        }
        return false;
    }

    static uint64_t NowMs() { return ecowarden::NowMs(); }
};

} // namespace ecowarden
