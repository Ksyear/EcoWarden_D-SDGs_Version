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
 * @file  json_packet.h
 * @brief 클러스터 + 이벤트 데이터를 nlohmann/json으로 직렬화하여 UDP 전송
 * @date  2026
 *
 * JSON 패킷 구조:
 * {
 *   "frame_id": 42,
 *   "timestamp": "2026-03-27T12:34:56.789Z",
 *   "clusters": [
 *     {
 *       "id": 0,
 *       "x": 1200.0,
 *       "y": 350.0,
 *       "count": 12,
 *       "type": "normal"        ← normal / abandoned
 *     }
 *   ],
 *   "event": null               ← 이벤트 없으면 null
 *   // 또는
 *   "event": {
 *     "type": "abandoned",
 *     "x": 800.0,
 *     "y": 200.0,
 *     "cluster_id": 1,
 *     "timestamp": "2026-03-27T12:34:56.789Z"
 *   }
 * }
 *
 * 최대 패킷 크기: 65507 bytes (UDP 최대 payload)
 * 초과 시 클러스터를 포인트 수 내림차순으로 정렬 후 잘라냄.
 */

#pragma once

#include "scan_processor.h"
#include "cluster_tracker.h"
#include "zone_policy.h"
#include "udp_sender.h"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>

namespace ecowarden {

// ── UDP JSON 최대 크기 ──────────────────────────────────────────────
static constexpr size_t UDP_MAX_DGRAM = 65507;

// ── JSON 패킷 빌더 + 송신기 ────────────────────────────────────────
class JsonPacketSender {
public:
    explicit JsonPacketSender(UdpSender& udp);

    /**
     * @brief 클러스터 + (옵션) 이탈 이벤트를 JSON 직렬화 후 UDP 전송
     *
     * @param clusters    현재 프레임 클러스터 목록
     * @param tracks      현재 트랙 목록
     * @param dep_events  이번 프레임의 이탈 이벤트
     * @param dump_events 이번 프레임의 투기 확정 이벤트
     * @param frame_id    프레임 번호
     * @return true: 전송 성공
     */
    /**
     * @param tracked_only      true: 트랙 기반 객체만 전송 (Unity용, 노이즈 제거)
     *                          false: 전체 클러스터 + 점구름 전송 (시각화기/디버그용)
     * @param intrusion_events  이번 프레임의 금지구역 침입 이벤트 (보안 감시 핵심).
     *                          departure 처럼 UDP 손실 대비 5프레임 반복 전송된다.
     */
    bool Send(const std::vector<Cluster>& clusters,
              const std::vector<Track>& tracks,
              const std::vector<DepartureEvent>& dep_events,
              const std::vector<DumpingEvent>& dump_events,
              uint32_t frame_id,
              bool tracked_only = false,
              const std::vector<IntrusionEvent>& intrusion_events = {});

    /**
     * @brief 클러스터 + 이벤트를 JSON으로 직렬화만 수행 (전송 없이)
     *        테스트/디버그용
     */
    static std::string Serialize(
        const std::vector<Cluster>& clusters,
        const std::vector<Track>& tracks,
        const std::vector<DepartureEvent>& dep_events,
        const std::vector<DumpingEvent>& dump_events,
        uint32_t frame_id,
        bool tracked_only = false,
        const std::vector<IntrusionEvent>& intrusion_events = {}
    );

    size_t GetSentCount()    const { return sent_count_; }
    size_t GetDropCount()    const { return drop_count_; }
    size_t GetTruncCount()   const { return trunc_count_; }

private:
    UdpSender& udp_;

    std::atomic<size_t> sent_count_{0};
    std::atomic<size_t> drop_count_{0};    // 전송 실패
    std::atomic<size_t> trunc_count_{0};   // 크기 초과로 클러스터 잘림

    // departure 는 단발 이벤트라 UDP 패킷 손실 시 Unity 에 humanPrefab
    // 잔상이 영구히 남는다. 이후 N프레임 동안 반복 전송해 손실을 흡수한다.
    // (Unity 는 이미 삭제된 ID 의 departure 중복 수신을 무시한다)
    static constexpr int kDepartureResendFrames = 5;
    struct PendingDeparture {
        DepartureEvent evt;
        int remaining;
    };
    std::vector<PendingDeparture> pending_departures_;

    // intrusion 도 단발 이벤트라 같은 이유로 반복 전송한다. 같은 사람의
    // 재알림은 ZonePolicy 가 10초 간격으로 throttle 하므로, Unity 는
    // (person_track_id, timestamp) 중복 수신을 무시하면 된다.
    static constexpr int kIntrusionResendFrames = 5;
    struct PendingIntrusion {
        IntrusionEvent evt;
        int remaining;
    };
    std::vector<PendingIntrusion> pending_intrusions_;

    // dumping 은 이 시스템에서 **가장 중요한 단발 이벤트**인데 v56 이전에는
    // 1회만 보냈다. UDP 패킷 하나가 유실되면 Unity 의 투기 애니메이션과
    // 증거 사진 표시가 통째로 사라지고, 수신 측에서는 복구할 방법이 없다.
    // departure/intrusion 과 같은 방식으로 N프레임 반복 전송한다.
    // (Unity 는 같은 ID 에 무적 시간을 두어 중복을 흡수한다)
    static constexpr int kDumpingResendFrames = 5;
    struct PendingDump {
        DumpingEvent evt;
        int remaining;
    };
    std::vector<PendingDump> pending_dumps_;

    /**
     * @brief epoch ms → ISO 8601 문자열
     */
    static std::string MsToIso8601(uint64_t epoch_ms);

    /**
     * @brief 클러스터의 이벤트 타입을 판별
     *        트랙 상태가 Departed인 경우 "abandoned", 그 외 "normal"
     */
    static std::string ResolveType(
        const Cluster& cluster,
        const std::vector<Track>& tracks
    );

    static uint64_t NowMs();
};

} // namespace ecowarden
