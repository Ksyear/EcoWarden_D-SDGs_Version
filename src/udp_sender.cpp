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
 * @file  udp_sender.cpp
 * @brief POSIX UDP 소켓으로 점구름 + 클러스터 데이터를 Unity에 전송
 * @date  2026
 */

#include "udp_sender.h"

#include <cstdio>
#include <cstring>
#include <chrono>
#include <algorithm>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

namespace ecowarden {

// ── sockaddr 저장용 (헤더에서 POSIX 헤더 의존 제거) ─────────────────
struct UdpSender::SockAddrStorage {
    struct sockaddr_in addr;
};

// ── 현재 시각 (밀리초) ──────────────────────────────────────────────
uint64_t UdpSender::NowMs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
        ).count()
    );
}

// ── 생성자 / 소멸자 ─────────────────────────────────────────────────
UdpSender::UdpSender(const UdpSenderConfig& cfg) : cfg_(cfg) {}

UdpSender::~UdpSender() {
    Close();
}

// ── Open ─────────────────────────────────────────────────────────────
bool UdpSender::Open() {
    if (sock_fd_ >= 0) return true;

    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::fprintf(stderr, "[UDP] socket() failed: %s\n", strerror(errno));
        return false;
    }

    addr_ = new SockAddrStorage{};
    std::memset(&addr_->addr, 0, sizeof(addr_->addr));
    addr_->addr.sin_family = AF_INET;
    addr_->addr.sin_port   = htons(cfg_.dest_port);

    if (inet_pton(AF_INET, cfg_.dest_ip.c_str(), &addr_->addr.sin_addr) <= 0) {
        std::fprintf(stderr, "[UDP] Invalid IP: %s\n", cfg_.dest_ip.c_str());
        close(sock_fd_);
        sock_fd_ = -1;
        delete addr_;
        addr_ = nullptr;
        return false;
    }

    std::printf("[UDP] Socket opened → %s:%u\n",
                cfg_.dest_ip.c_str(), cfg_.dest_port);
    return true;
}

// ── Close ────────────────────────────────────────────────────────────
void UdpSender::Close() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
        std::printf("[UDP] Socket closed. "
                    "Total: %zu packets, %zu bytes, %zu errors\n",
                    total_packets_.load(), total_bytes_.load(),
                    send_errors_.load());
    }
    delete addr_;
    addr_ = nullptr;
}

// ── SendRaw: 바이트 배열 전송 ───────────────────────────────────────
bool UdpSender::SendRaw(const void* data, size_t len) {
    if (sock_fd_ < 0 || !addr_) return false;

    ssize_t sent = sendto(
        sock_fd_,
        data, len,
        0,
        reinterpret_cast<const struct sockaddr*>(&addr_->addr),
        sizeof(addr_->addr)
    );

    if (sent < 0) {
        send_errors_++;
        // 전송 실패를 1000회마다 로그 (폭주 방지)
        if (send_errors_ == 1 || send_errors_ % 1000 == 0) {
            std::fprintf(stderr, "[UDP] sendto() failed: %s (total errors=%zu)\n",
                         strerror(errno), send_errors_.load());
        }
        return false;
    }

    total_packets_++;
    total_bytes_ += static_cast<size_t>(sent);
    return true;
}

// ── SendBuffer: 공개 전송 인터페이스 ─────────────────────────────────
bool UdpSender::SendBuffer(const void* data, size_t len) {
    if (len > 65507) {
        std::fprintf(stderr, "[UDP] Packet too large: %zu > 65507\n", len);
        return false;
    }
    return SendRaw(data, len);
}

// ── SendPoints: 점구름 분할 전송 ────────────────────────────────────
//
//   450 points → ceil(450/153) = 3 packets
//   packet 0: header + points[0..152]
//   packet 1: header + points[153..305]
//   packet 2: header + points[306..449]
//
size_t UdpSender::SendPoints(const std::vector<CartesianPoint>& points,
                             uint32_t frame_id) {
    if (points.empty() || sock_fd_ < 0) return 0;

    const size_t total = points.size();
    const size_t frag_count =
        (total + MAX_POINTS_PER_PKT - 1) / MAX_POINTS_PER_PKT;

    // 패킷 버퍼 (스택에 할당 — 최대 1400 bytes)
    uint8_t buf[UDP_MAX_PAYLOAD];
    uint64_t ts = NowMs();
    size_t packets_sent = 0;

    for (size_t frag = 0; frag < frag_count; ++frag) {
        size_t offset = frag * MAX_POINTS_PER_PKT;
        size_t count  = std::min(MAX_POINTS_PER_PKT, total - offset);

        // 헤더 구성
        PacketHeader* hdr = reinterpret_cast<PacketHeader*>(buf);
        hdr->magic          = UDP_MAGIC;
        hdr->type           = PKT_TYPE_POINTS;
        hdr->flags          = 0;
        hdr->frame_id       = frame_id;
        hdr->timestamp_ms   = ts;
        hdr->fragment_index = static_cast<uint8_t>(frag);
        hdr->fragment_count = static_cast<uint8_t>(frag_count);
        hdr->item_count     = static_cast<uint16_t>(count);

        // 포인트 데이터 복사
        PointItem* items = reinterpret_cast<PointItem*>(buf + HEADER_SIZE);
        for (size_t i = 0; i < count; ++i) {
            const auto& pt = points[offset + i];
            items[i].x_mm      = static_cast<float>(pt.x_mm);
            items[i].y_mm      = static_cast<float>(pt.y_mm);
            items[i].intensity  = pt.intensity;
        }

        size_t pkt_size = HEADER_SIZE + count * sizeof(PointItem);
        if (SendRaw(buf, pkt_size)) {
            packets_sent++;
        }
    }

    return packets_sent;
}

// ── SendClusters: 클러스터 + 트랙 메타데이터 전송 ───────────────────
//
//   클러스터별로 가장 가까운 트랙을 매칭하여 track_id, track_state,
//   velocity를 함께 전송한다. Unity 측에서 객체 시각화에 활용.
//
size_t UdpSender::SendClusters(const std::vector<Cluster>& clusters,
                               const std::vector<Track>& tracks,
                               uint32_t frame_id) {
    if (clusters.empty() || sock_fd_ < 0) return 0;

    const size_t total = clusters.size();
    const size_t frag_count =
        (total + MAX_CLUSTERS_PER_PKT - 1) / MAX_CLUSTERS_PER_PKT;

    uint8_t buf[UDP_MAX_PAYLOAD];
    uint64_t ts = NowMs();
    size_t packets_sent = 0;

    for (size_t frag = 0; frag < frag_count; ++frag) {
        size_t offset = frag * MAX_CLUSTERS_PER_PKT;
        size_t count  = std::min(MAX_CLUSTERS_PER_PKT, total - offset);

        PacketHeader* hdr = reinterpret_cast<PacketHeader*>(buf);
        hdr->magic          = UDP_MAGIC;
        hdr->type           = PKT_TYPE_CLUSTERS;
        hdr->flags          = 0;
        hdr->frame_id       = frame_id;
        hdr->timestamp_ms   = ts;
        hdr->fragment_index = static_cast<uint8_t>(frag);
        hdr->fragment_count = static_cast<uint8_t>(frag_count);
        hdr->item_count     = static_cast<uint16_t>(count);

        ClusterItem* items = reinterpret_cast<ClusterItem*>(buf + HEADER_SIZE);

        for (size_t i = 0; i < count; ++i) {
            const auto& cl = clusters[offset + i];
            ClusterItem& ci = items[i];

            ci.cluster_id     = static_cast<uint16_t>(offset + i);
            ci.centroid_x_mm  = static_cast<float>(cl.centroid_x_mm);
            ci.centroid_y_mm  = static_cast<float>(cl.centroid_y_mm);
            ci.point_count    = static_cast<uint16_t>(cl.points.size());

            // 클러스터 centroid에 가장 가까운 트랙 찾기
            ci.track_id       = 0;
            ci.track_state    = 0xFF;  // untracked
            ci.velocity_x_mm  = 0.0f;
            ci.velocity_y_mm  = 0.0f;

            double best_dist_sq = 1e18;
            for (const auto& tr : tracks) {
                if (tr.state == TrackState::Lost) continue;
                if (!tr.confirmed && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
                if (tr.lost_count > 0 && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
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
                double dx = cl.centroid_x_mm - tr.x_mm;
                double dy = cl.centroid_y_mm - tr.y_mm;
                double d2 = dx * dx + dy * dy;
                if (d2 < best_dist_sq) {
                    best_dist_sq    = d2;
                    ci.track_id     = tr.person_group_id ? tr.person_group_id : tr.id;
                    ci.track_state  = static_cast<uint8_t>(tr.state);
                    ci.velocity_x_mm = static_cast<float>(tr.x_mm - tr.prev_x_mm);
                    ci.velocity_y_mm = static_cast<float>(tr.y_mm - tr.prev_y_mm);
                }
            }
        }

        size_t pkt_size = HEADER_SIZE + count * sizeof(ClusterItem);
        if (SendRaw(buf, pkt_size)) {
            packets_sent++;
        }
    }

    return packets_sent;
}

} // namespace ecowarden
