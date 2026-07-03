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
 * @file  udp_sender.h
 * @brief 점구름 + 클러스터/트랙 데이터를 Unity로 UDP 전송
 * @date  2026
 *
 * 바이너리 프로토콜 (little-endian, packed):
 *
 * ┌─────────────── PacketHeader (20 bytes) ────────────────┐
 * │ magic (2)  0x4C44 ("LD")                               │
 * │ type  (1)  0x01=points, 0x02=clusters                  │
 * │ flags (1)  reserved                                    │
 * │ frame_id       (4)  프레임 번호                         │
 * │ timestamp_ms   (8)  epoch 밀리초                        │
 * │ fragment_index (1)  현재 조각 번호 (0-based)            │
 * │ fragment_count (1)  총 조각 수                          │
 * │ item_count     (2)  이 패킷 내 아이템 수                │
 * └────────────────────────────────────────────────────────┘
 *
 * ── type 0x01: PointItem (9 bytes × N) ──────────────────
 * │ x_mm       (float32)                                   │
 * │ y_mm       (float32)                                   │
 * │ intensity  (uint8)                                     │
 *
 * ── type 0x02: ClusterItem (21 bytes × N) ───────────────
 * │ cluster_id   (uint16)                                  │
 * │ centroid_x   (float32)                                 │
 * │ centroid_y   (float32)                                 │
 * │ point_count  (uint16)                                  │
 * │ track_id     (uint32)                                  │
 * │ track_state  (uint8)  0=moving,1=stop,2=departed,      │
 * │                       3=lost, 0xFF=untracked           │
 * │ velocity_x   (float32) mm/frame 추정 속도              │
 * │ velocity_y   (float32)                                 │
 *
 * MTU 안전 범위: 패킷 당 최대 1400 bytes payload
 *   → 포인트: (1400-20)/9 = 153 points/packet
 *   → 클러스터: (1400-20)/21 = 65 clusters/packet
 */

#pragma once

#include "scan_processor.h"
#include "cluster_tracker.h"

#include <cstdint>
#include <string>
#include <vector>
#include <atomic>

namespace ecowarden {

// ── 패킷 상수 ───────────────────────────────────────────────────────
static constexpr uint16_t UDP_MAGIC          = 0x4C44; // "LD"
static constexpr uint8_t  PKT_TYPE_POINTS    = 0x01;
static constexpr uint8_t  PKT_TYPE_CLUSTERS  = 0x02;
static constexpr size_t   UDP_MAX_PAYLOAD    = 1400;
static constexpr size_t   HEADER_SIZE        = 20;

// ── 패킷 헤더 (packed, 20 bytes) ────────────────────────────────────
#pragma pack(push, 1)

struct PacketHeader {
    uint16_t magic;           // 0x4C44
    uint8_t  type;            // PKT_TYPE_POINTS or PKT_TYPE_CLUSTERS
    uint8_t  flags;           // reserved (0)
    uint32_t frame_id;
    uint64_t timestamp_ms;
    uint8_t  fragment_index;  // 0-based
    uint8_t  fragment_count;  // 총 조각 수
    uint16_t item_count;      // 이 패킷 내 아이템 수
};
static_assert(sizeof(PacketHeader) == 20, "PacketHeader must be 20 bytes");

struct PointItem {
    float   x_mm;
    float   y_mm;
    uint8_t intensity;
};
static_assert(sizeof(PointItem) == 9, "PointItem must be 9 bytes");

struct ClusterItem {
    uint16_t cluster_id;
    float    centroid_x_mm;
    float    centroid_y_mm;
    uint16_t point_count;
    uint32_t track_id;
    uint8_t  track_state;     // TrackState enum or 0xFF
    float    velocity_x_mm;   // mm/frame
    float    velocity_y_mm;
};
static_assert(sizeof(ClusterItem) == 25, "ClusterItem must be 25 bytes");

#pragma pack(pop)

// ── 포인트/클러스터 최대 수 (패킷 당) ───────────────────────────────
static constexpr size_t MAX_POINTS_PER_PKT   =
    (UDP_MAX_PAYLOAD - HEADER_SIZE) / sizeof(PointItem);    // 153
static constexpr size_t MAX_CLUSTERS_PER_PKT =
    (UDP_MAX_PAYLOAD - HEADER_SIZE) / sizeof(ClusterItem);  // 65

// ── UDP 전송 설정 ────────────────────────────────────────────────────
struct UdpSenderConfig {
  std::string dest_ip = "192.168.20.173";
  uint16_t dest_port = 5005;
};


// ── UDP 송신기 ───────────────────────────────────────────────────────
class UdpSender {
public:
    explicit UdpSender(const UdpSenderConfig& cfg = UdpSenderConfig{});
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;

    /**
     * @brief UDP 소켓을 생성한다.
     * @return true: 성공
     */
    bool Open();

    /**
     * @brief 소켓을 닫는다.
     */
    void Close();

    bool IsOpen() const { return sock_fd_ >= 0; }

    /**
     * @brief 한 프레임의 점구름을 전송한다 (자동 분할).
     * @param points   직교좌표 포인트 벡터
     * @param frame_id 프레임 번호
     * @return 전송된 패킷 수
     */
    size_t SendPoints(const std::vector<CartesianPoint>& points,
                      uint32_t frame_id);

    /**
     * @brief 클러스터 + 트랙 메타데이터를 전송한다.
     * @param clusters  DBSCAN 클러스터 결과
     * @param tracks    현재 트랙 목록 (클러스터와 매칭)
     * @param frame_id  프레임 번호
     * @return 전송된 패킷 수
     */
    size_t SendClusters(const std::vector<Cluster>& clusters,
                        const std::vector<Track>& tracks,
                        uint32_t frame_id);

    /**
     * @brief 임의 바이트 버퍼를 UDP로 전송한다 (JSON 패킷 등).
     *        65507 bytes 초과 시 전송하지 않고 false 반환.
     */
    bool SendBuffer(const void* data, size_t len);

    size_t GetTotalPackets() const { return total_packets_; }
    size_t GetTotalBytes()   const { return total_bytes_; }
    size_t GetSendErrors()   const { return send_errors_; }

private:
    UdpSenderConfig cfg_;
    int             sock_fd_ = -1;

    // 소켓 주소 (Open 시 초기화)
    struct SockAddrStorage;
    SockAddrStorage* addr_ = nullptr;

    std::atomic<size_t> total_packets_{0};
    std::atomic<size_t> total_bytes_{0};
    std::atomic<size_t> send_errors_{0};

    static uint64_t NowMs();

    bool SendRaw(const void* data, size_t len);
};

} // namespace ecowarden
