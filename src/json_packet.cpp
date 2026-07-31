/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - RplidarS2 LiDAR 센서 인터페이스 및 객체 탐지
 */

#include "json_packet.h"
#include "time_utils.h"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <cstring>

using json = nlohmann::json;

namespace ecowarden {

JsonPacketSender::JsonPacketSender(UdpSender& udp) : udp_(udp) {}

std::string JsonPacketSender::MsToIso8601(uint64_t epoch_ms) {
    return ecowarden::MsToIso8601(epoch_ms);
}
uint64_t JsonPacketSender::NowMs() { return ecowarden::NowMs(); }

std::string JsonPacketSender::ResolveType(const Cluster& cluster,
                                          const std::vector<Track>& tracks) {
    if (tracks.empty()) return "normal";
    double best_sq = 1e18;
    TrackState best_state = TrackState::Moving;
    bool is_dumped = false;
    for (const auto& tr : tracks) {
        double dx = cluster.centroid_x_mm - tr.x_mm;
        double dy = cluster.centroid_y_mm - tr.y_mm;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_sq) {
            best_sq = d2;
            best_state = tr.state;
            is_dumped = tr.is_dumped_item;
        }
    }
    if (is_dumped) return "dumped";
    return (best_state == TrackState::Departed) ? "abandoned" : "normal";
}

std::string JsonPacketSender::Serialize(
    const std::vector<Cluster>& clusters,
    const std::vector<Track>& tracks,
    const std::vector<DepartureEvent>& dep_events,
    const std::vector<DumpingEvent>& dump_events,
    uint32_t frame_id,
    bool tracked_only,
    const std::vector<IntrusionEvent>& intrusion_events)
{
    uint64_t now = NowMs();
    json j_objects = json::array();
    json j_points = json::array();

    if (tracked_only) {
        // ── Unity 모드: tracks 기반 객체만 전송 (고스트/노이즈 제거) ──
        for (const auto& tr : tracks) {
            if (tr.state == TrackState::Lost) continue;
            if (!tr.confirmed && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
            if (tr.lost_count > 0 && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
            if (tr.is_dump_suspect && !tr.is_dumped_item) continue;
            if (tr.is_object_candidate &&
                !tr.is_dump_suspect &&
                !tr.is_dumped_item &&
                tr.person_group_id == 0) continue;
            if (IsUngroupedStationaryLargeObject(tr)) continue;
            if (IsBackgroundResidualTrack(tr)) continue;
            if (IsHeldPersonLeg(tr)) continue;
            if (tr.person_group_id != 0 &&
                !tr.person_group_primary &&
                !tr.is_dump_suspect &&
                !tr.is_dumped_item) continue;

            const char* type_str = "normal";
            if (tr.is_dumped_item)  type_str = "dumped";
            else if (tr.is_dump_suspect) type_str = "suspect";
            else if (tr.is_object_candidate) type_str = "object_candidate";
            else if (tr.state == TrackState::Departed) type_str = "abandoned";

            json j_obj;
            const double out_x = tr.person_group_id ? tr.person_group_x_mm : tr.x_mm;
            const double out_y = tr.person_group_id ? tr.person_group_y_mm : tr.y_mm;
            const double out_w = tr.person_group_id ? tr.person_group_width_mm : tr.width_mm;
            j_obj["id"]    = tr.person_group_id ? tr.person_group_id : tr.id;
            j_obj["x"]     = std::round(out_x * 10.0) / 10.0;
            j_obj["y"]     = std::round(out_y * 10.0) / 10.0;
            j_obj["type"]  = type_str;
            j_obj["width"] = std::round(out_w * 10.0) / 10.0;
            j_obj["is_abandoned"] = tr.is_dumped_item || tr.state == TrackState::Departed;
            if (tr.person_group_id) {
                j_obj["person_group_id"] = tr.person_group_id;
            }
            j_objects.push_back(std::move(j_obj));
        }
        // points 비어 있음 — Unity는 트랙 위치만 사용
    } else {
        // ── 시각화기 모드: 전체 클러스터 + 점구름 전송 ──
        int pt_count = 0;
        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& cl = clusters[i];

            for (const auto& p : cl.points) {
                if (pt_count++ % 4 == 0) {
                    j_points.push_back({
                        static_cast<int>(std::round(p.x_mm)),
                        static_cast<int>(std::round(p.y_mm))
                    });
                }
            }

            double best_sq = 1e18;
            const Track* best_track = nullptr;
            for (const auto& tr : tracks) {
                double dx = cl.centroid_x_mm - tr.x_mm;
                double dy = cl.centroid_y_mm - tr.y_mm;
                double d2 = dx * dx + dy * dy;
                if (d2 < best_sq) {
                    best_sq = d2;
                    best_track = &tr;
                }
            }

            const char* type_str = "normal";
            if (best_track && best_sq <= 400.0 * 400.0) {
                if (best_track->is_dumped_item) type_str = "dumped";
                else if (best_track->state == TrackState::Departed) type_str = "abandoned";
            }

            json j_cl;
            j_cl["id"]    = static_cast<int>(i);
            j_cl["x"]     = std::round(cl.centroid_x_mm * 10.0) / 10.0;
            j_cl["y"]     = std::round(cl.centroid_y_mm * 10.0) / 10.0;
            j_cl["type"]  = type_str;
            j_cl["width"] = std::round(cl.width_mm * 10.0) / 10.0;
            j_cl["is_abandoned"] = (std::strcmp(type_str, "abandoned") == 0 || std::strcmp(type_str, "dumped") == 0);
            j_objects.push_back(std::move(j_cl));
        }
    }

    json j_tracks = json::array();
    for (const auto& tr : tracks) {
        if (tr.state == TrackState::Lost) continue;
        if (!tr.confirmed && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
        if (tr.lost_count > 0 && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
        if (tracked_only &&
            tr.is_object_candidate &&
            !tr.is_dump_suspect &&
            !tr.is_dumped_item &&
            tr.person_group_id == 0) continue;
        if (tracked_only && IsUngroupedStationaryLargeObject(tr)) continue;
        if (tracked_only && IsBackgroundResidualTrack(tr)) continue;
        if (tracked_only && IsHeldPersonLeg(tr)) continue;
        if (tr.person_group_id != 0 &&
            !tr.person_group_primary &&
            !tr.is_dump_suspect &&
            !tr.is_dumped_item) continue;
        json j_tr;
        const double out_x = tr.person_group_id ? tr.person_group_x_mm : tr.x_mm;
        const double out_y = tr.person_group_id ? tr.person_group_y_mm : tr.y_mm;
        j_tr["id"]    = tr.person_group_id ? tr.person_group_id : tr.id;
        j_tr["x"]     = std::round(out_x * 10.0) / 10.0;
        j_tr["y"]     = std::round(out_y * 10.0) / 10.0;
        const char* state_str = "unknown";
        switch (tr.state) {
            case TrackState::Tentative:  state_str = "tentative"; break;
            case TrackState::Moving:     state_str = "moving"; break;
            case TrackState::Stationary: state_str = "stationary"; break;
            case TrackState::Departed:   state_str = "departed"; break;
            case TrackState::Lost:       state_str = "lost"; break;
        }
        j_tr["state"] = state_str;
        j_tr["is_dumped_item"] = tr.is_dumped_item;
        j_tr["is_dump_suspect"] = tr.is_dump_suspect;
        j_tr["is_object_candidate"] = tr.is_object_candidate;
        if (tr.person_group_id) {
            j_tr["person_group_id"] = tr.person_group_id;
            j_tr["group_width"] = std::round(tr.person_group_width_mm * 10.0) / 10.0;
        }
        j_tracks.push_back(std::move(j_tr));
    }

    json j_events = json::array();
    for (const auto& evt : intrusion_events) {
        json j_evt;
        j_evt["type"] = "intrusion";
        j_evt["person_track_id"] = evt.person_track_id;
        j_evt["person_x"] = std::round(evt.person_x_mm * 10.0) / 10.0;
        j_evt["person_y"] = std::round(evt.person_y_mm * 10.0) / 10.0;
        j_evt["zone"] = evt.zone;
        j_evt["severity"] = "high";
        j_evt["timestamp"] = evt.timestamp_ms;
        j_events.push_back(std::move(j_evt));
    }
    for (const auto& evt : dump_events) {
        json j_evt;
        j_evt["type"] = "dumping";
        j_evt["person_track_id"] = evt.person_track_id;
        j_evt["person_x"] = std::round(evt.person_x_mm * 10.0) / 10.0;
        j_evt["person_y"] = std::round(evt.person_y_mm * 10.0) / 10.0;
        j_evt["object_track_id"] = evt.object_track_id;
        j_evt["object_x"] = std::round(evt.object_x_mm * 10.0) / 10.0;
        j_evt["object_y"] = std::round(evt.object_y_mm * 10.0) / 10.0;
        j_evt["timestamp"] = evt.timestamp_ms;
        j_events.push_back(std::move(j_evt));
    }
    for (const auto& evt : dep_events) {
        json j_evt;
        j_evt["type"] = "departure";
        j_evt["person_track_id"] = 0;
        j_evt["person_x"] = 0.0;
        j_evt["person_y"] = 0.0;
        j_evt["object_track_id"] = evt.track_id;
        j_evt["object_x"] = std::round(evt.x_mm * 10.0) / 10.0;
        j_evt["object_y"] = std::round(evt.y_mm * 10.0) / 10.0;
        j_evt["timestamp"] = evt.timestamp_ms;
        j_events.push_back(std::move(j_evt));
    }
    if (tracked_only) {
        std::vector<uint32_t> hidden_ids;
        for (const auto& tr : tracks) {
            if (tr.is_dumped_item) continue;
            if (!tr.confirmed) continue;

            bool hidden = false;
            uint32_t hidden_id = tr.id;
            if (tr.is_dump_suspect) {
                hidden = true;
            } else if (tr.lost_count > 0 || tr.state == TrackState::Lost) {
                hidden = true;
                if (tr.person_group_id != 0 && tr.person_group_primary) {
                    hidden_id = tr.person_group_id;
                }
            } else if (tr.is_object_candidate && tr.person_group_id == 0) {
                hidden = true;
            } else if (IsUngroupedStationaryLargeObject(tr)) {
                hidden = true;
            } else if (IsBackgroundResidualTrack(tr)) {
                hidden = true;
            } else if (IsHeldPersonLeg(tr)) {
                hidden = true;
            } else if (tr.person_group_id != 0 && !tr.person_group_primary) {
                hidden = true;
                hidden_id = tr.id;
            }
            if (!hidden) continue;
            if (std::find(hidden_ids.begin(), hidden_ids.end(), hidden_id) != hidden_ids.end()) {
                continue;
            }
            hidden_ids.push_back(hidden_id);

            json j_evt;
            j_evt["type"] = "departure";
            j_evt["person_track_id"] = 0;
            j_evt["person_x"] = 0.0;
            j_evt["person_y"] = 0.0;
            j_evt["object_track_id"] = hidden_id;
            j_evt["object_x"] = std::round(tr.x_mm * 10.0) / 10.0;
            j_evt["object_y"] = std::round(tr.y_mm * 10.0) / 10.0;
            j_evt["timestamp"] = now;
            j_events.push_back(std::move(j_evt));
        }
    }

    json packet;
    packet["type"]      = "FRAME";
    packet["frame_id"]  = frame_id;
    packet["timestamp"] = now;
    packet["objects"]   = std::move(j_objects);
    packet["tracks"]    = std::move(j_tracks);
    packet["points"]    = std::move(j_points);
    packet["events"]    = std::move(j_events);
    return packet.dump();
}

bool JsonPacketSender::Send(const std::vector<Cluster>& clusters,
                            const std::vector<Track>& tracks,
                            const std::vector<DepartureEvent>& dep_events,
                            const std::vector<DumpingEvent>& dump_events,
                            uint32_t frame_id,
                            bool tracked_only,
                            const std::vector<IntrusionEvent>& intrusion_events)
{
    // departure 재전송: 직전 프레임들에서 발생한 departure 를 이번 패킷에도
    // 포함한다. 신규 departure 는 이번 Serialize 에 이미 들어가므로
    // 다음 프레임부터 (kDepartureResendFrames - 1)회 더 나간다.
    std::vector<DepartureEvent> dep_with_pending = dep_events;
    for (auto& pd : pending_departures_) {
        dep_with_pending.push_back(pd.evt);
        pd.remaining--;
    }
    pending_departures_.erase(
        std::remove_if(pending_departures_.begin(), pending_departures_.end(),
                       [](const PendingDeparture& pd) { return pd.remaining <= 0; }),
        pending_departures_.end());
    for (const auto& evt : dep_events) {
        pending_departures_.push_back({evt, kDepartureResendFrames - 1});
    }

    // intrusion 재전송: departure 와 동일한 UDP 손실 흡수 정책.
    std::vector<IntrusionEvent> intr_with_pending = intrusion_events;
    for (auto& pi : pending_intrusions_) {
        intr_with_pending.push_back(pi.evt);
        pi.remaining--;
    }
    pending_intrusions_.erase(
        std::remove_if(pending_intrusions_.begin(), pending_intrusions_.end(),
                       [](const PendingIntrusion& pi) { return pi.remaining <= 0; }),
        pending_intrusions_.end());
    for (const auto& evt : intrusion_events) {
        pending_intrusions_.push_back({evt, kIntrusionResendFrames - 1});
    }

    std::string payload = Serialize(clusters, tracks, dep_with_pending, dump_events, frame_id, tracked_only, intr_with_pending);
    if (payload.size() <= UDP_MAX_DGRAM) {
        if (udp_.SendBuffer(payload.data(), payload.size())) {
            sent_count_++;
            return true;
        }
        return false;
    }
    return false;
}

} // namespace ecowarden
