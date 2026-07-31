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
 * @file  phase_profiler.h
 * @brief 메인 루프 phase 별 latency 측정기 (header-only)
 *
 * v5 에서 신설. v4까지는 phase 별 실측 비용을 측정할 수 없어 scratch 영속화
 * (PERF-5) 같은 최적화의 실제 이득을 수치화할 수 없었다. 본 프로파일러는
 * steady_clock 기반으로 매 프레임 각 phase 의 소요 시간(us)을 CSV 로 기록한다.
 *
 * ── 활성화 ──────────────────────────────────────────────────────────
 *   export ECOWARDEN_PROFILE_CSV=/tmp/ecowarden_phases.csv
 *   ./rplidar_app ...
 * 환경 변수가 없으면 모든 호출이 no-op (오버헤드 ≈ steady_clock::now() 1회).
 *
 * ── CSV 스키마 ──────────────────────────────────────────────────────
 *   frame, scan_us, filter_us, bg_us, tracker_us, dump_us,
 *          events_us, json_us, http_us, total_us, kf_uncert_max
 *
 *   - *_us : 해당 phase 의 delta us
 *   - total_us : BeginFrame 으로부터 EndFrame 까지의 wall 시간
 *   - kf_uncert_max : 모든 트랙의 KF PositionUncertaintySq() 최대값
 *                     (v5 [3 CAUTION] — 장기 발산 감시용)
 *
 * ── 롤링 요약 ───────────────────────────────────────────────────────
 *   PrintRollingSummary(N) 를 매 프레임 호출하면 N 프레임마다 stderr 로
 *   각 phase 의 p50/p95 + total 평균 + FPS 를 한 줄 출력한다.
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>

namespace ecowarden {

class PhaseProfiler {
public:
    enum Phase {
        kScan = 0,   // lidar.GetScanFrame
        kFilter,     // processor.Process
        kBackground, // bg_filter.Apply
        kTracker,    // tracker.UpdateTracking
        kDump,       // dumper.Process (post S2)
        kEvents,     // notifier + suspect cleanup
        kJsonUdp,    // json_sender.Send
        kHttpCamera, // (현재 측정 안 함 — 비동기)
        kPhaseCount
    };

    PhaseProfiler() {
        if (const char* p = std::getenv("ECOWARDEN_PROFILE_CSV")) {
            csv_ = std::fopen(p, "w");
            if (csv_) {
                std::fprintf(csv_,
                    "frame,scan_us,filter_us,bg_us,tracker_us,"
                    "dump_us,events_us,json_us,http_us,total_us,kf_uncert_max\n");
                enabled_ = true;
            } else {
                std::fprintf(stderr,
                    "[PROF] failed to open %s (no profiling)\n", p);
            }
        }
    }

    ~PhaseProfiler() {
        if (csv_) {
            std::fflush(csv_);
            std::fclose(csv_);
        }
    }

    PhaseProfiler(const PhaseProfiler&) = delete;
    PhaseProfiler& operator=(const PhaseProfiler&) = delete;

    bool Enabled() const { return enabled_; }

    void BeginFrame() {
        if (!enabled_) return;
        t_begin_ = std::chrono::steady_clock::now();
        t_prev_  = t_begin_;
        phase_us_.fill(0.0);
    }

    // 이전 Mark (또는 BeginFrame) 으로부터 경과 시간을 phase p 에 귀속.
    void Mark(Phase p) {
        if (!enabled_) return;
        auto now = std::chrono::steady_clock::now();
        double us =
            std::chrono::duration<double, std::micro>(now - t_prev_).count();
        phase_us_[p] += us;
        t_prev_ = now;
    }

    // 프레임 종료. kf_uncert_max 는 호출자가 구해서 건네준다.
    void EndFrame(double kf_uncert_max) {
        if (!enabled_) return;
        auto now = std::chrono::steady_clock::now();
        double total_us =
            std::chrono::duration<double, std::micro>(now - t_begin_).count();

        frame_no_++;
        std::fprintf(csv_,
            "%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.6f\n",
            static_cast<unsigned long>(frame_no_),
            phase_us_[kScan], phase_us_[kFilter], phase_us_[kBackground],
            phase_us_[kTracker], phase_us_[kDump], phase_us_[kEvents],
            phase_us_[kJsonUdp], phase_us_[kHttpCamera],
            total_us, kf_uncert_max);

        // 롤링 윈도 (phase + total)
        PushRolling(kPhaseCount, total_us);
        for (int i = 0; i < kPhaseCount; ++i) {
            PushRolling(static_cast<Phase>(i), phase_us_[i]);
        }
    }

    // N 프레임마다 stderr 로 p50/p95 한 줄 요약.
    void PrintRollingSummary(int every_n_frames) {
        if (!enabled_) return;
        if (every_n_frames <= 0) return;
        if (frame_no_ == 0 || frame_no_ % every_n_frames != 0) return;

        double p50_total = Percentile(kPhaseCount, 0.50);
        double p95_total = Percentile(kPhaseCount, 0.95);
        double fps = (p50_total > 0.0) ? (1e6 / p50_total) : 0.0;

        std::fprintf(stderr,
            "[PROF] f=%lu total p50=%.0fus p95=%.0fus (~%.1f fps) | "
            "scan=%.0f filter=%.0f bg=%.0f trk=%.0f dump=%.0f "
            "evt=%.0f json=%.0f\n",
            static_cast<unsigned long>(frame_no_), p50_total, p95_total, fps,
            Percentile(kScan, 0.50), Percentile(kFilter, 0.50),
            Percentile(kBackground, 0.50), Percentile(kTracker, 0.50),
            Percentile(kDump, 0.50), Percentile(kEvents, 0.50),
            Percentile(kJsonUdp, 0.50));
    }

private:
    static constexpr size_t kRollingWindow = 300;

    void PushRolling(int idx, double v) {
        auto& dq = rolling_[idx];
        dq.push_back(v);
        if (dq.size() > kRollingWindow) dq.pop_front();
    }

    // 0..kPhaseCount-1 은 phase, kPhaseCount 는 total 을 의미.
    double Percentile(int idx, double pct) const {
        const auto& dq = rolling_[idx];
        if (dq.empty()) return 0.0;
        std::vector<double> v(dq.begin(), dq.end());
        std::sort(v.begin(), v.end());
        size_t k = static_cast<size_t>(pct * (v.size() - 1));
        return v[k];
    }

    bool enabled_ = false;
    FILE* csv_ = nullptr;
    std::chrono::steady_clock::time_point t_begin_{};
    std::chrono::steady_clock::time_point t_prev_{};
    std::array<double, kPhaseCount> phase_us_{};
    uint64_t frame_no_ = 0;
    // phase 0..kPhaseCount-1 + total (인덱스 kPhaseCount)
    std::array<std::deque<double>, kPhaseCount + 1> rolling_{};
};

} // namespace ecowarden
