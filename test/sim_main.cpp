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
 * @file  sim_main.cpp
 * @brief sim 회귀 스위트 — production 파이프라인을 합성 시나리오로 검증
 *
 * v5 (2026-04-11) 전면 재작성:
 *
 * 이전(v4 이하): 시나리오 1개를 main() 안에 하드코딩하고 콘솔에 출력만 함.
 *                 sim 자체 파라미터가 main.cpp 와 drift 했으며 pass/fail 판정이 없음.
 *                 → "데모"이지 회귀 스위트가 아니었음.
 *
 * v5: 4 시나리오 (walk-by / drop-and-leave / 2-people / occlusion) 를 데이터로
 *     표현하고 동일 러너로 모두 실행. production_params.h 의 단일 파라미터 셋으로
 *     ScanProcessor / BackgroundFilter / ClusterTracker 를 구성 — main.cpp 와 byte
 *     identical. 각 시나리오는 ScenarioOracle 로 자동 PASS/FAIL 판정. 종료 코드는
 *     실패 시나리오 수.
 *
 *     본 sim 의 통과 = v5 머지 게이트. 알고리즘 변경 (예: DumpDetector 분리) 시
 *     이 sim 이 동일 결과를 내야 함.
 *
 * 빌드:
 *   cd build && cmake .. && make -j$(nproc)
 * 실행:
 *   ./rplidar_sim                # 전체 4 시나리오
 *   ./rplidar_sim --only=B       # B_drop_and_leave 만 실행
 *   ./rplidar_sim --verbose      # 매 frame 트랙/이벤트 상세 출력
 */

#ifndef __SANE_USERSPACE_TYPES__
#define __SANE_USERSPACE_TYPES__
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <vector>

#include "background_filter.h"
#include "cluster_tracker.h"
#include "dump_validation.h"
#include "phase_profiler.h"
#include "production_params.h"
#include "scan_processor.h"
#include "zone_policy.h"

// ── Ctrl+C 처리 ─────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;
static void SignalHandler(int) { g_running = 0; }

// ════════════════════════════════════════════════════════════════════
//   합성 엔티티 / 시나리오 표현
// ════════════════════════════════════════════════════════════════════

namespace sim {

struct Entity {
    double x_mm;     // centroid X (mm, sensor 기준)
    double y_mm;     // centroid Y (mm)
    double width_mm; // 클러스터 폭
    int    npts;    // 점 개수 (5=사람 다리, 4=봉투 등)
};

// 한 frame 의 장면을 (frame_no) → entities 함수로 표현.
using SceneFunc = std::vector<Entity> (*)(int frame);

struct Oracle {
    // dump_events 의 정확 개수. -1 이면 무관.
    int exact_dump = -1;
    // dep_events 누적 카운트가 [min, max] 범위.
    int dep_min = 0;
    int dep_max = 1000;
    // 첫 dump_event 발생 frame 이 [first_min, first_max] 범위. 0 이면 무관.
    int first_dump_min = 0;
    int first_dump_max = 0;
    // 시나리오 동안 동시 active track 의 최대치 하한.
    int peak_tracks_min = 1;
    // 종료 시 active track 상한. -1 이면 무관.
    int final_tracks_max = -1;
    // Unity tracked-only 송신 기준 객체 수. -1 이면 무관.
    int peak_unity_objects_min = -1;
    int peak_unity_objects_max = -1;
    int final_unity_objects_max = -1;
};

struct Scenario {
    const char* name;
    int total_frames; // 배경 학습 50 frame 포함
    SceneFunc scene;
    Oracle oracle;
};

// ── jitter 헬퍼 (deterministic per scenario+frame) ──────────────────
//   같은 시나리오 + 같은 frame 은 항상 같은 jitter 값을 반환한다.
//   회귀 비교 시 결정성이 깨지면 안 된다.
static uint32_t HashSeed(const char* name, int frame, int salt) {
    uint32_t h = 2166136261u;  // FNV-1a
    for (const char* p = name; *p; ++p) {
        h ^= static_cast<uint8_t>(*p);
        h *= 16777619u;
    }
    h ^= static_cast<uint32_t>(frame);    h *= 16777619u;
    h ^= static_cast<uint32_t>(salt);     h *= 16777619u;
    return h;
}

// 합성 클러스터를 ScanFrame 에 추가한다.
//   - 거리/각도 jitter (±[8, 18] mm), intensity jitter (80~120) 를 결정성 있게 부여
//   - 이는 BackgroundFilter 의 seen_count 임계와 상호작용 시 합성 신호가
//     너무 균질해서 false negative 가 나는 문제를 완화한다.
static void AddClusterPoints(const Entity& e, const char* scn_name, int frame,
                             ecowarden::ScanFrame& out) {
    const double dist  = std::sqrt(e.x_mm * e.x_mm + e.y_mm * e.y_mm);
    if (dist < 1.0) return;
    const double angle = std::atan2(e.y_mm, e.x_mm);
    const double half  = std::atan2(e.width_mm / 2.0, dist);

    for (int i = 0; i < e.npts; i++) {
        const double t = (e.npts <= 1)
            ? 0.0
            : (static_cast<double>(i) / (e.npts - 1) * 2.0 - 1.0);
        const double a = angle + t * half;

        // deterministic jitter
        const uint32_t s_d = HashSeed(scn_name, frame, i * 3 + 1);
        const uint32_t s_i = HashSeed(scn_name, frame, i * 3 + 2);
        const double depth_jit = 8.0 + (s_d % 100) / 10.0;          // [8.0, 17.9]
        const double sign      = (i % 2 == 0) ? 1.0 : -1.0;
        const double d         = dist + sign * depth_jit;
        const uint8_t intens   = static_cast<uint8_t>(80 + (s_i % 41));  // [80, 120]

        float deg = static_cast<float>(a * 180.0 / M_PI);
        if (deg < 0)   deg += 360.0f;
        if (deg > 360) deg -= 360.0f;

        ecowarden::ScanPoint sp;
        sp.angle_deg   = deg;
        sp.distance_mm = static_cast<uint16_t>(std::max(1.0, d));
        sp.intensity   = intens;
        out.push_back(sp);
    }
}

// ════════════════════════════════════════════════════════════════════
//   4 시나리오 정의
// ════════════════════════════════════════════════════════════════════

// ── 좌표 규약 ───────────────────────────────────────────────────────
//   FOV = 0~180° (atan2(y, x)) → y 는 항상 양수여야 한다.
//   X = 좌우, Y = 전방 깊이 (positive = 앞)
//
//   사람 = width 300mm, 6 points
//   봉투 = width 120mm, 4 points (min_cluster_width=50 통과)

// ── A: walk-by ──────────────────────────────────────────────────────
//   사람 1명이 FOV 를 좌→우 가로지름. 투기 없음.
//   frame 1~50  : 배경 학습 (빈 frame)
//   frame 51~120: 사람 Y=1800 (고정), X -2500 → +3500 (step 100mm, 60 frames)
//   frame 121~135: 범위 이탈 후 ID 삭제 확인
static std::vector<Entity> SceneA(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 120) {
        double x = -2500.0 + (f - 51) * 100.0;
        if (x < 3500.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
    }
    return ents;
}

// ── B: drop-and-leave ───────────────────────────────────────────────
//   사람 1명 진입 → 봉투를 떨굼 → 멀어짐
//
//   트릭: 트래커 association 이 greedy distance 매칭이므로, 봉투가 사람의 이전
//         위치와 너무 가까우면 사람 트랙이 봉투에 "납치" 된다. 봉투를 Y 방향으로
//         300mm offset 해서 사람 cluster 가 이전 track pos 에 더 가깝도록 강제.
//
//   frame 51~75 : 사람 Y=1800, X -2000→+500 (step 100/frame, 25 frames, 2500mm 걸음)
//   frame 76    : 사람 X=650 Y=1800 (150mm 앞으로), 봉투 (500, 1500) 출현 (∵ 300mm Y offset)
//   frame 77~130: 사람 X 150mm/frame 로 계속 멀어짐, 봉투 정지
static std::vector<Entity> SceneB(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 75) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 130) {
        double x = 650.0 + (f - 76) * 150.0;  // 150mm/frame 떠남
        if (x < 6500.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
        // 봉투 — Y=1350 (사람보다 450mm 앞) 에 정지. merge_radius=350 이상 이격 강제.
        ents.push_back({500.0, 1350.0, 100.0, 8});
    }
    return ents;
}

// ── C: 2-people ─────────────────────────────────────────────────────
//   두 사람이 다른 Y 에서 반대 방향으로 통과. 투기 없음.
//   frame 51~120 : 사람1 Y=1500, X -2500→ +4500 (step 100)
//   frame 60~120 : 사람2 Y=2500, X +3000→-3000 (step -100)
static std::vector<Entity> SceneC(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 120) {
        double x1 = -2500.0 + (f - 51) * 100.0;
        if (x1 < 4500.0) ents.push_back({x1, 1500.0, 200.0, 12});
    }
    if (f >= 60 && f <= 120) {
        double x2 = 3000.0 - (f - 60) * 100.0;
        if (x2 > -3000.0) ents.push_back({x2, 2500.0, 200.0, 12});
    }
    return ents;
}

// ── D: 10-frame occlusion ───────────────────────────────────────────
//   사람이 10 frame 동안 사라졌다가 재등장. 투기 없음.
//   lost_age_limit=15 이라 10 frame occlusion 후 재등장 시 새 트랙일 수도 있지만
//   첫 구간이 min_walk_dist (500mm) 를 초과하므로 새 트랙이 되더라도
//   dep_events 1건 남기고 dump 0 이면 OK.
//   frame 51~80 : 사람 Y=1800, X -2000→+1000 (step 100, 30 frames)
//   frame 81~90 : 사람 숨김 (occlusion 10 frames)
//   frame 91~120: 사람 Y=1800, X +1200→+4200 재등장 (step 100, 30 frames)
static std::vector<Entity> SceneD(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 80) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 91 && f <= 120) {
        double x = 1200.0 + (f - 91) * 100.0;
        if (x < 4500.0) ents.push_back({x, 1800.0, 200.0, 12});
    }
    return ents;
}

// ── E: hotspot_traffic ────────────────────────────────────────────────
//   고빈도 통행 + Hotspot: 사람이 투기 후 동일 지역을 반복 통과해도 FP 없음.
//   frame 51~90 : 사람1 투기 (B와 동일 패턴)
//   frame 91~130: Hotspot 등록 상태에서 사람2가 같은 지역 통과 → dump=0 (사람2에 한해)
static std::vector<Entity> SceneE(int f) {
    std::vector<Entity> ents;
    // 사람1: 투기 시나리오
    if (f >= 51 && f <= 75) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 90) {
        double x = 650.0 + (f - 76) * 150.0;
        if (x < 6500.0) ents.push_back({x, 1800.0, 200.0, 12});
        ents.push_back({500.0, 1350.0, 100.0, 8}); // 봉투
    }
    // 사람2: hotspot 지역 통과 (frame 100~150, Y=1400 — 봉투 근처)
    if (f >= 100 && f <= 150) {
        double x = -2000.0 + (f - 100) * 120.0;
        if (x < 5000.0) ents.push_back({x, 1400.0, 200.0, 12});
    }
    // 봉투는 사람1 떠난 후에도 계속 존재
    if (f >= 91 && f <= 160) {
        ents.push_back({500.0, 1350.0, 100.0, 8});
    }
    return ents;
}

// ── F: background_change ─────────────────────────────────────────────
//   배경 학습 후 새 고정 물체 등장. 적응형 배경이 FP 없이 학습하는지 검증.
//   frame 51~80: 사람 Y=1800 라인 통과
//   frame 90~250: 고정 물체 등장 (Y=3500, 사람 궤적과 완전 분리)
//   frame 200~250: 사람2 Y=3600 라인 통과 (고정 물체 근처)
static std::vector<Entity> SceneF(int f) {
    std::vector<Entity> ents;
    // 사람1: Y=1800 라인 (고정 물체와 멀리 떨어짐)
    if (f >= 51 && f <= 80) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    }
    // 고정 물체 (쓰레기통): Y=3500, 사람1/2 궤적에서 멀리
    if (f >= 90) {
        ents.push_back({0.0, 3500.0, 400.0, 15});
    }
    // 사람2: Y=3600 라인 (고정 물체 근처지만 궤적 매칭 거리 밖)
    if (f >= 200 && f <= 250) {
        double x = -2500.0 + (f - 200) * 100.0;
        ents.push_back({x, 3600.0, 200.0, 12});
    }
    return ents;
}

// ── G: frame_jitter ──────────────────────────────────────────────────
//   동일한 walk-by이지만 entity 위치에 큰 jitter를 부여.
//   KF가 jitter를 흡수하고 안정적 매칭을 유지하는지 검증.
static std::vector<Entity> SceneG(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 120) {
        double base_x = -2500.0 + (f - 51) * 100.0;
        // 큰 jitter: ±30mm X, ±20mm Y (deterministic)
        uint32_t h = HashSeed("G_jitter", f, 42);
        double jx = (static_cast<int>(h % 61) - 30);         // [-30, 30]
        double jy = (static_cast<int>((h >> 8) % 41) - 20);  // [-20, 20]
        if (base_x + jx < 3500.0) {
            ents.push_back({base_x + jx, 1800.0 + jy, 200.0, 12});
        }
    }
    return ents;
}

// ── H: multi_dump ────────────────────────────────────────────────────
//   두 사람이 각각 다른 위치에서 투기. 동시 다발 감지.
//   사람1: frame 51~90 진입 + 투기 → 멀어짐
//   사람2: frame 70~110 진입 + 투기 → 멀어짐
static std::vector<Entity> SceneH(int f) {
    std::vector<Entity> ents;
    // 사람1: Y=1500 라인
    if (f >= 51 && f <= 75) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1500.0, 200.0, 12});
    } else if (f >= 76 && f <= 130) {
        double x = 650.0 + (f - 76) * 150.0;
        if (x < 6500.0) ents.push_back({x, 1500.0, 200.0, 12});
        ents.push_back({500.0, 1100.0, 100.0, 8}); // 봉투1
    }
    // 사람2: Y=2500 라인
    if (f >= 70 && f <= 95) {
        double x = 2000.0 - (f - 70) * 100.0;
        ents.push_back({x, 2500.0, 200.0, 12});
    } else if (f >= 96 && f <= 140) {
        double x = -650.0 - (f - 96) * 150.0;
        if (x > -6500.0) ents.push_back({x, 2500.0, 200.0, 12});
        ents.push_back({-500.0, 2100.0, 100.0, 8}); // 봉투2
    }
    return ents;
}

// ── I: large_split ───────────────────────────────────────────────────
//   두 사람이 같이 걷다가 분리. split이 dump FP 되면 안 됨.
//   frame 51~80: 두 사람 나란히 같은 방향으로 이동 (X 간격 250mm)
//   frame 81~120: 사람1 왼쪽으로, 사람2 오른쪽으로 분리 (둘 다 계속 이동)
static std::vector<Entity> SceneI(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 80) {
        // 두 사람 나란히 같은 방향 이동
        double base_x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({base_x, 1800.0, 200.0, 12});
        ents.push_back({base_x + 250.0, 1800.0, 200.0, 12});
    } else if (f >= 81 && f <= 120) {
        // 사람1: 왼쪽으로 계속 이동
        double x1 = 1000.0 - (f - 81) * 100.0;
        if (x1 > -3000.0) ents.push_back({x1, 1800.0, 200.0, 12});
        // 사람2: 오른쪽으로 계속 이동
        double x2 = 1250.0 + (f - 81) * 100.0;
        if (x2 < 5000.0) ents.push_back({x2, 1800.0, 200.0, 12});
    }
    return ents;
}

// ── J: sensor_spike ──────────────────────────────────────────────────
//   정상 보행 중 노이즈 spike (한 프레임에만 거대 클러스터 출현).
//   시스템이 spike 를 무시하고 안정적으로 추적을 유지하는지 검증.
static std::vector<Entity> SceneJ(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 120) {
        double x = -2500.0 + (f - 51) * 100.0;
        if (x < 3500.0) ents.push_back({x, 1800.0, 200.0, 12});
    }
    // 노이즈 spike: frame 70, 85 에 거대+초소형 클러스터 출현
    if (f == 70 || f == 85) {
        ents.push_back({3000.0, 3000.0, 2000.0, 20}); // 초대형 → width 필터에서 제거되어야 함
        ents.push_back({-500.0, 500.0, 20.0, 2});       // 초소형 → 점수 부족으로 제거
    }
    return ents;
}

// ── K: reflective_spike_near_path ───────────────────────────────────
//   사람 경로 근처에 2~3프레임 반사 cluster가 생겨도 dump/suspect 확정이 없어야 한다.
static std::vector<Entity> SceneK(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 120) {
        double x = -2500.0 + (f - 51) * 100.0;
        if (x < 3500.0) ents.push_back({x, 1800.0, 200.0, 12});
    }
    if (f >= 70 && f <= 72) {
        ents.push_back({-300.0, 1450.0, 70.0, 4});
    }
    if (f >= 88 && f <= 89) {
        ents.push_back({1200.0, 2100.0, 80.0, 4});
    }
    return ents;
}

// ── L: background_edge_jitter ───────────────────────────────────────
//   고정 배경 edge가 20~40프레임 흔들린 뒤 사람이 지나가도 dump=0.
static std::vector<Entity> SceneL(int f) {
    std::vector<Entity> ents;
    if (f >= 60 && f <= 100) {
        const double jx = (f % 2 == 0) ? -150.0 : 150.0;
        ents.push_back({jx, 3200.0, 90.0, 4});
    }
    if (f >= 105 && f <= 155) {
        double x = -2500.0 + (f - 105) * 100.0;
        ents.push_back({x, 3000.0, 200.0, 12});
    }
    return ents;
}

// ── M: preexisting_object_passersby ─────────────────────────────────
//   오래 존재한 정지 객체 앞을 사람이 지나가도 새 투기로 확정되면 안 된다.
static std::vector<Entity> SceneM(int f) {
    std::vector<Entity> ents;
    if (f >= 51) {
        ents.push_back({0.0, 2400.0, 120.0, 6});
    }
    if (f >= 150 && f <= 210) {
        double x = -2500.0 + (f - 150) * 100.0;
        ents.push_back({x, 2450.0, 200.0, 12});
    }
    return ents;
}

// ── N: real_drop_with_noise ─────────────────────────────────────────
//   실제 투기와 주변 반사 spike가 동시에 있어도 dump=1, source 오염 없음.
static std::vector<Entity> SceneN(int f) {
    std::vector<Entity> ents = SceneB(f);
    if (f >= 76 && f <= 78) {
        ents.push_back({900.0, 1000.0, 70.0, 4});
    }
    if (f == 90 || f == 91) {
        ents.push_back({-700.0, 1700.0, 70.0, 4});
    }
    return ents;
}

// ── O: quick_exit_drop ──────────────────────────────────────────────
//   투기 직후 source가 빠르게 시야 밖으로 사라져도 source-lost 경로로 확정되어야 한다.
static std::vector<Entity> SceneO(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 75) {
        double x = -1800.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 82) {
        double x = 650.0 + (f - 76) * 220.0;
        if (x < 1900.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
        ents.push_back({500.0, 1350.0, 100.0, 8});
    } else if (f >= 83 && f <= 130) {
        ents.push_back({500.0, 1350.0, 100.0, 8});
    }
    return ents;
}

static std::vector<Entity> SceneDropObject(int f, double width_mm, int npts) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 75) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 150) {
        double x = 650.0 + (f - 76) * 150.0;
        if (x < 6500.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
        ents.push_back({500.0, 1350.0, width_mm, npts});
    }
    return ents;
}

// ── P: small cup / small bag drop ───────────────────────────────────
static std::vector<Entity> SceneP(int f) {
    return SceneDropObject(f, 70.0, 4);
}

// ── Q: 20L bag-sized drop ───────────────────────────────────────────
static std::vector<Entity> SceneQ(int f) {
    return SceneDropObject(f, 220.0, 6);
}

// ── R: 100L bag-sized drop ──────────────────────────────────────────
static std::vector<Entity> SceneR(int f) {
    return SceneDropObject(f, 800.0, 12);
}

// ── S: preexisting large bag passersby ──────────────────────────────
static std::vector<Entity> SceneS(int f) {
    std::vector<Entity> ents;
    if (f >= 51) {
        ents.push_back({0.0, 2400.0, 800.0, 12});
    }
    if (f >= 150 && f <= 230) {
        double x = -2500.0 + (f - 150) * 100.0;
        ents.push_back({x, 2450.0, 200.0, 12});
    }
    return ents;
}

// ── T: stationary/slow person is not trash ──────────────────────────
static std::vector<Entity> SceneT(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 150) {
        double x = (f < 90) ? 0.0 : (f - 90) * 8.0;
        ents.push_back({x, 1800.0, 400.0, 14});
    }
    return ents;
}

// ── U: close delayed split drop ─────────────────────────────────────
//   쓰레기가 사람 바로 옆에서 늦게 분리되어도 너무 오래 hold되면 안 된다.
static std::vector<Entity> SceneU(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 75) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 150) {
        double x = 650.0 + (f - 76) * 40.0;
        if (x < 3600.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
        ents.push_back({560.0, 1740.0, 120.0, 6});
    }
    return ents;
}

// ── V: very quick throw-and-exit ────────────────────────────────────
//   source가 투기 직후 2~3프레임만 보이고 사라져도 확정되어야 한다.
static std::vector<Entity> SceneV(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 75) {
        double x = -1800.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 78) {
        double x = 650.0 + (f - 76) * 280.0;
        if (x < 1300.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
        ents.push_back({500.0, 1350.0, 100.0, 8});
    } else if (f >= 79 && f <= 130) {
        ents.push_back({500.0, 1350.0, 100.0, 8});
    }
    return ents;
}

// ── W: wide stride one person ───────────────────────────────────────
//   한 사람의 두 다리가 1300mm 간격으로 별도 cluster가 되어도 Unity에는 1객체.
static std::vector<Entity> SceneW(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 120) {
        double x = -2200.0 + (f - 51) * 80.0;
        if (x < 3500.0) {
            ents.push_back({x - 650.0, 1800.0, 90.0, 5});
            ents.push_back({x + 650.0, 1800.0, 90.0, 5});
        }
    }
    return ents;
}

// ── X: slow stride, one foot almost stationary ──────────────────────
//   느린 보행에서 한쪽 다리가 정지 물체처럼 보여도 사람 group은 1객체 유지.
static std::vector<Entity> SceneX(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 130) {
        double base = -1200.0 + (f - 51) * 35.0;
        double phase = ((f / 8) % 2 == 0) ? 0.0 : 140.0;
        ents.push_back({base - 350.0, 1800.0, 90.0, 5});
        ents.push_back({base + 350.0 + phase, 1800.0, 90.0, 5});
    }
    return ents;
}

// ── Y: close people must not merge ──────────────────────────────────
//   가까운 두 사람이 지나갈 때 wide-stride 완화가 1명으로 오병합하면 안 된다.
static std::vector<Entity> SceneY(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 125) {
        double x = -2200.0 + (f - 51) * 75.0;
        if (x < 3600.0) {
            ents.push_back({x, 1450.0, 280.0, 12});
            ents.push_back({x + 120.0, 2450.0, 280.0, 12});
        }
    }
    return ents;
}

// ── Z: close attached drop must split from person ───────────────────
//   쓰레기가 사람 바로 옆에서 떨어져도 scan merge가 사람+쓰레기를 1객체로 붙이면 안 된다.
static std::vector<Entity> SceneZ(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 75) {
        double x = -2000.0 + (f - 51) * 100.0;
        ents.push_back({x, 1800.0, 200.0, 12});
    } else if (f >= 76 && f <= 140) {
        double x = 650.0 + (f - 76) * 110.0;
        if (x < 5000.0) {
            ents.push_back({x, 1800.0, 200.0, 12});
        }
        ents.push_back({520.0, 1670.0, 100.0, 8});
    }
    return ents;
}

// ── AA: missed background residual must not become Unity object ─────
//   배경 학습 직후 생긴 중간 크기 정적 잔상 앞을 사람이 지나가도
//   잔상이 독립 객체/투기물로 승격되면 안 된다.
static std::vector<Entity> SceneAA(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 220) {
        ents.push_back({0.0, 2800.0, 300.0, 8});
    }
    if (f >= 120 && f <= 170) {
        const double x = -2400.0 + (f - 120) * 95.0;
        ents.push_back({x, 2800.0, 240.0, 12});
    }
    return ents;
}

// ── BB: stationary crowd must not become dumping ───────────────────
//   여러 사람이 걸어 들어와 가까운 위치에서 같이 정지해도
//   사람 간 간격/궤적을 쓰레기 이탈로 오해하면 안 된다.
static std::vector<Entity> SceneBB(int f) {
    std::vector<Entity> ents;
    if (f >= 51 && f <= 180) {
        const double a = (f < 82) ? (-2100.0 + (f - 51) * 60.0) : -240.0;
        const double b = (f < 82) ? (-1500.0 + (f - 51) * 60.0) : 360.0;
        const double c = (f < 92) ? (2100.0 - (f - 51) * 55.0) : -155.0;
        ents.push_back({a, 1800.0, 260.0, 12});
        ents.push_back({b, 1800.0, 260.0, 12});
        ents.push_back({c, 2450.0, 260.0, 12});
    }
    return ents;
}

static const Scenario kScenarios[] = {
    {
        "A_walk_by", 135, &SceneA,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 3,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1, /*final_tracks_max*/ 0 }
    },
    {
        "B_drop_and_leave", 130, &SceneB,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 130,
                /*peak_tracks_min*/ 2 }
    },
    {
        "C_two_people", 120, &SceneC,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 2 }
    },
    {
        "D_occlusion", 120, &SceneD,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "E_hotspot_traffic", 160, &SceneE,
        // 사람1 투기 1건 + 사람2 통과 시 추가 dump 없어야 함. 총 dump=1.
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 100,
                /*peak_tracks_min*/ 2 }
    },
    {
        "F_background_change", 260, &SceneF,
        // 고정 물체 등장해도 dump=0. 사람2 통과해도 dump=0.
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "G_frame_jitter", 120, &SceneG,
        // jitter 있어도 walk-by → dump=0.
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 3,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "H_multi_dump", 140, &SceneH,
        // 두 사람 각각 투기 → dump=2.
        Oracle{ /*exact_dump*/ 2, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 140,
                /*peak_tracks_min*/ 2 }
    },
    {
        "I_large_split", 120, &SceneI,
        // 나란히 걷다가 분리 → dump=0 (split 이 FP 되면 안 됨).
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "J_sensor_spike", 120, &SceneJ,
        // 노이즈 spike 에도 안정 추적 → dump=0.
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 3,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "K_reflective_spike_near_path", 120, &SceneK,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 3,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "L_background_edge_jitter", 160, &SceneL,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "M_preexisting_object_passersby", 220, &SceneM,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "N_real_drop_with_noise", 130, &SceneN,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 130,
                /*peak_tracks_min*/ 2 }
    },
    {
        "O_quick_exit_drop", 130, &SceneO,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 130,
                /*peak_tracks_min*/ 2 }
    },
    {
        "P_small_cup_drop", 150, &SceneP,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 150,
                /*peak_tracks_min*/ 2 }
    },
    {
        "Q_20L_bag_drop", 150, &SceneQ,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 150,
                /*peak_tracks_min*/ 2 }
    },
    {
        "R_100L_bag_drop", 150, &SceneR,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 150,
                /*peak_tracks_min*/ 2 }
    },
    {
        "S_large_preexisting_bag_passersby", 240, &SceneS,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 2 }
    },
    {
        "T_stationary_person_not_trash", 150, &SceneT,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1 }
    },
    {
        "U_close_delayed_split_drop", 150, &SceneU,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 130,
                /*peak_tracks_min*/ 2 }
    },
    {
        "V_very_quick_throw_exit", 130, &SceneV,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 125,
                /*peak_tracks_min*/ 2 }
    },
    {
        "W_wide_stride_one_person", 135, &SceneW,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 2, /*final_tracks_max*/ -1,
                /*peak_unity_objects_min*/ 1, /*peak_unity_objects_max*/ 1 }
    },
    {
        "X_slow_stride_one_foot_stationary", 140, &SceneX,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 2, /*final_tracks_max*/ -1,
                /*peak_unity_objects_min*/ 1, /*peak_unity_objects_max*/ 1 }
    },
    {
        "Y_two_people_close_not_merged", 135, &SceneY,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 2, /*final_tracks_max*/ -1,
                /*peak_unity_objects_min*/ 2, /*peak_unity_objects_max*/ 2 }
    },
    {
        "Z_close_attached_drop_split", 150, &SceneZ,
        Oracle{ /*exact_dump*/ 1, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 80, /*first_dump_max*/ 140,
                /*peak_tracks_min*/ 2 }
    },
    {
        "AA_background_residual_passby", 235, &SceneAA,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 5,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 1, /*final_tracks_max*/ -1,
                /*peak_unity_objects_min*/ 1, /*peak_unity_objects_max*/ 1,
                /*final_unity_objects_max*/ 0 }
    },
    {
        "BB_stationary_crowd_no_dump", 180, &SceneBB,
        Oracle{ /*exact_dump*/ 0, /*dep_min*/ 0, /*dep_max*/ 8,
                /*first_dump_min*/ 0, /*first_dump_max*/ 0,
                /*peak_tracks_min*/ 3, /*final_tracks_max*/ -1,
                /*peak_unity_objects_min*/ 2, /*peak_unity_objects_max*/ 3 }
    },
};
static constexpr size_t kNumScenarios = sizeof(kScenarios) / sizeof(kScenarios[0]);

// ════════════════════════════════════════════════════════════════════
//   결과 / 러너
// ════════════════════════════════════════════════════════════════════

struct ScenarioResult {
    const Scenario* sc;
    int total_dump = 0;
    int total_dep  = 0;
    int first_dump_frame = -1;
    int peak_tracks = 0;
    int final_tracks = 0;
    int peak_unity_objects = 0;
    int final_unity_objects = 0;
    bool pass = false;
    std::string fail_reason;
};

static int CountActiveTracks(const std::vector<ecowarden::Track>& tracks) {
    int n = 0;
    for (const auto& t : tracks) {
        if (t.state != ecowarden::TrackState::Lost) n++;
    }
    return n;
}

static int CountUnityObjects(const std::vector<ecowarden::Track>& tracks) {
    std::vector<uint32_t> ids;
    for (const auto& tr : tracks) {
        if (tr.state == ecowarden::TrackState::Lost) continue;
        if (!tr.confirmed && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
        if (tr.lost_count > 0 && !tr.is_dump_suspect && !tr.is_dumped_item) continue;
        if (tr.is_dump_suspect && !tr.is_dumped_item) continue;
        if (tr.is_object_candidate &&
            !tr.is_dump_suspect &&
            !tr.is_dumped_item &&
            tr.person_group_id == 0) continue;
        if (ecowarden::IsUngroupedStationaryLargeObject(tr)) continue;
        if (ecowarden::IsBackgroundResidualTrack(tr)) continue;
        if (ecowarden::IsHeldPersonLeg(tr)) continue;
        if (tr.person_group_id != 0 &&
            !tr.person_group_primary &&
            !tr.is_dump_suspect &&
            !tr.is_dumped_item) continue;

        const uint32_t out_id = tr.person_group_id ? tr.person_group_id : tr.id;
        if (std::find(ids.begin(), ids.end(), out_id) == ids.end()) {
            ids.push_back(out_id);
        }
    }
    return static_cast<int>(ids.size());
}

static double MaxKfUncert(const std::vector<ecowarden::Track>& tracks) {
    double m = 0.0;
    for (const auto& t : tracks) {
        if (t.kf.IsInitialized()) {
            double u = t.kf.PositionUncertaintySq();
            if (u > m) m = u;
        }
    }
    return m;
}

static ScenarioResult RunScenario(const Scenario& sc, bool verbose,
                                   ecowarden::PhaseProfiler* prof) {
    ScenarioResult res;
    res.sc = &sc;

    // 매 시나리오마다 파이프라인을 새로 생성 (배경 학습 상태 리셋)
    ecowarden::ScanProcessor    processor (ecowarden::DefaultFilterParams(),
                                            ecowarden::DefaultDBSCANParams());
    ecowarden::BackgroundFilter bg_filter (ecowarden::DefaultBackgroundFilterParams());
    ecowarden::ClusterTracker   tracker   (ecowarden::DefaultTrackerParams());

    for (int f = 1; f <= sc.total_frames && g_running; f++) {
        if (prof) prof->BeginFrame();
        // (1) 합성 ScanFrame 생성
        ecowarden::ScanFrame frame;
        // 배경 학습용 잡음을 frame 에 깔아주기 위해, 학습 단계에서도
        // 빈 frame 이 아닌 "빈 공간" 을 보내야 한다. 하지만 BackgroundFilter 는
        // 클러스터를 학습하므로, 학습 단계에서 클러스터가 0 이면 학습이 진전되지
        // 않는다. 이를 피하기 위해 학습 단계 / 운용 단계 모두 동일한 entity 함수를
        // 호출하지만 학습 단계에는 어떤 클러스터도 안 나오므로 그냥 빈 ScanFrame 을
        // 통과시킨다 — BackgroundFilter::Apply 가 frame_count_ 만 증가시키고
        // 자연스럽게 학습 단계를 마친다.
        const auto entities = sc.scene(f);
        for (const auto& e : entities) {
            AddClusterPoints(e, sc.name, f, frame);
        }

        if (prof) prof->Mark(ecowarden::PhaseProfiler::kScan);

        // (2) ScanProcessor → 클러스터
        std::vector<ecowarden::Cluster> clusters;
        processor.Process(frame, clusters);
        if (prof) prof->Mark(ecowarden::PhaseProfiler::kFilter);

        // (3) BackgroundFilter
        const bool ready = bg_filter.Apply(clusters);
        if (!ready) {
            if (verbose && f % 10 == 0) {
                std::printf("    [BG] learning %u/%u\n",
                            bg_filter.GetFrameCount(),
                            ecowarden::DefaultBackgroundFilterParams().learning_frames);
            }
            if (prof) {
                prof->Mark(ecowarden::PhaseProfiler::kBackground);
                prof->EndFrame(MaxKfUncert(tracker.GetTracks()));
            }
            continue;
        }
        if (prof) prof->Mark(ecowarden::PhaseProfiler::kBackground);

        // (4) ClusterTracker
        std::vector<ecowarden::DepartureEvent> dep_events;
        std::vector<ecowarden::DumpingEvent>   dump_events;
        tracker.Update(clusters, dep_events, dump_events);
        if (prof) {
            prof->Mark(ecowarden::PhaseProfiler::kTracker);
            prof->EndFrame(MaxKfUncert(tracker.GetTracks()));
        }

        // (5) 통계 누적
        res.total_dep  += static_cast<int>(dep_events.size());
        res.total_dump += static_cast<int>(dump_events.size());
        if (!dump_events.empty() && res.first_dump_frame < 0) {
            res.first_dump_frame = f;
        }
        const int active = CountActiveTracks(tracker.GetTracks());
        if (active > res.peak_tracks) res.peak_tracks = active;
        res.final_tracks = active;
        const int unity_objects = CountUnityObjects(tracker.GetTracks());
        if (unity_objects > res.peak_unity_objects) {
            res.peak_unity_objects = unity_objects;
        }
        res.final_unity_objects = unity_objects;

        if (verbose) {
            std::printf("  f=%3d clusters=%zu tracks=%zu unity=%d dep=%zu dump=%zu",
                        f, clusters.size(), tracker.GetTracks().size(),
                        unity_objects, dep_events.size(), dump_events.size());
            for (const auto& tr : tracker.GetTracks()) {
                const char* tag = "N";
                if (tr.is_dumped_item)   tag = "D";
                else if (tr.is_dump_suspect) tag = "S";
                std::printf(" [%s#%u@(%.0f,%.0f) age=%u stc=%u cum=%.0f]",
                            tag, tr.id, tr.x_mm, tr.y_mm, tr.age,
                            tr.stationary_count, tr.cumulative_dist_mm);
            }
            std::printf("\n");
        }
    }

    // (6) Oracle 평가
    const auto& o = sc.oracle;
    bool ok = true;
    char buf[256];

    if (o.exact_dump >= 0 && res.total_dump != o.exact_dump) {
        std::snprintf(buf, sizeof(buf),
                      "dump_events=%d (expected exact %d)",
                      res.total_dump, o.exact_dump);
        res.fail_reason = buf; ok = false;
    } else if (res.total_dep < o.dep_min || res.total_dep > o.dep_max) {
        std::snprintf(buf, sizeof(buf),
                      "dep_events=%d (expected [%d,%d])",
                      res.total_dep, o.dep_min, o.dep_max);
        res.fail_reason = buf; ok = false;
    } else if (o.first_dump_min > 0 &&
               (res.first_dump_frame < o.first_dump_min ||
                res.first_dump_frame > o.first_dump_max)) {
        std::snprintf(buf, sizeof(buf),
                      "first_dump=frame %d (expected [%d,%d])",
                      res.first_dump_frame, o.first_dump_min, o.first_dump_max);
        res.fail_reason = buf; ok = false;
    } else if (res.peak_tracks < o.peak_tracks_min) {
        std::snprintf(buf, sizeof(buf),
                      "peak_tracks=%d (expected ≥ %d)",
                      res.peak_tracks, o.peak_tracks_min);
        res.fail_reason = buf; ok = false;
    } else if (o.final_tracks_max >= 0 && res.final_tracks > o.final_tracks_max) {
        std::snprintf(buf, sizeof(buf),
                      "final_tracks=%d (expected ≤ %d)",
                      res.final_tracks, o.final_tracks_max);
        res.fail_reason = buf; ok = false;
    } else if (o.peak_unity_objects_min >= 0 &&
               res.peak_unity_objects < o.peak_unity_objects_min) {
        std::snprintf(buf, sizeof(buf),
                      "peak_unity_objects=%d (expected ≥ %d)",
                      res.peak_unity_objects, o.peak_unity_objects_min);
        res.fail_reason = buf; ok = false;
    } else if (o.peak_unity_objects_max >= 0 &&
               res.peak_unity_objects > o.peak_unity_objects_max) {
        std::snprintf(buf, sizeof(buf),
                      "peak_unity_objects=%d (expected ≤ %d)",
                      res.peak_unity_objects, o.peak_unity_objects_max);
        res.fail_reason = buf; ok = false;
    } else if (o.final_unity_objects_max >= 0 &&
               res.final_unity_objects > o.final_unity_objects_max) {
        std::snprintf(buf, sizeof(buf),
                      "final_unity_objects=%d (expected ≤ %d)",
                      res.final_unity_objects, o.final_unity_objects_max);
        res.fail_reason = buf; ok = false;
    }

    res.pass = ok;
    return res;
}

static void PrintScenarioResult(const ScenarioResult& r) {
    const auto& o = r.sc->oracle;
    std::printf("─────────────────────────────────────────────────────\n");
    std::printf("SCENARIO: %s (%d frames)\n", r.sc->name, r.sc->total_frames);
    std::printf("─────────────────────────────────────────────────────\n");

    auto print_line = [](const char* label, const char* value, bool ok) {
        std::printf("  %-14s %-30s %s\n", label, value, ok ? "PASS" : "FAIL");
    };

    char buf[128];

    // dep_events
    std::snprintf(buf, sizeof(buf), "%d (oracle [%d, %d])",
                  r.total_dep, o.dep_min, o.dep_max);
    print_line("dep_events:", buf, r.total_dep >= o.dep_min && r.total_dep <= o.dep_max);

    // dump_events
    if (o.exact_dump >= 0) {
        std::snprintf(buf, sizeof(buf), "%d (oracle exact %d)",
                      r.total_dump, o.exact_dump);
        print_line("dump_events:", buf, r.total_dump == o.exact_dump);
    } else {
        std::snprintf(buf, sizeof(buf), "%d (oracle any)", r.total_dump);
        print_line("dump_events:", buf, true);
    }

    // first_dump
    if (o.first_dump_min > 0) {
        if (r.first_dump_frame < 0) {
            std::snprintf(buf, sizeof(buf), "(none) (oracle [%d, %d])",
                          o.first_dump_min, o.first_dump_max);
            print_line("first_dump:", buf, false);
        } else {
            std::snprintf(buf, sizeof(buf), "frame %d (oracle [%d, %d])",
                          r.first_dump_frame, o.first_dump_min, o.first_dump_max);
            print_line("first_dump:", buf,
                       r.first_dump_frame >= o.first_dump_min &&
                       r.first_dump_frame <= o.first_dump_max);
        }
    }

    // peak_tracks
    std::snprintf(buf, sizeof(buf), "%d (oracle ≥ %d)",
                  r.peak_tracks, o.peak_tracks_min);
    print_line("peak_tracks:", buf, r.peak_tracks >= o.peak_tracks_min);

    if (o.final_tracks_max >= 0) {
        std::snprintf(buf, sizeof(buf), "%d (oracle ≤ %d)",
                      r.final_tracks, o.final_tracks_max);
        print_line("final_tracks:", buf, r.final_tracks <= o.final_tracks_max);
    }

    if (o.peak_unity_objects_min >= 0 || o.peak_unity_objects_max >= 0) {
        char range[32];
        if (o.peak_unity_objects_min >= 0 && o.peak_unity_objects_max >= 0) {
            std::snprintf(range, sizeof(range), "[%d, %d]",
                          o.peak_unity_objects_min, o.peak_unity_objects_max);
        } else if (o.peak_unity_objects_min >= 0) {
            std::snprintf(range, sizeof(range), "≥ %d", o.peak_unity_objects_min);
        } else {
            std::snprintf(range, sizeof(range), "≤ %d", o.peak_unity_objects_max);
        }
        const bool ok =
            (o.peak_unity_objects_min < 0 ||
             r.peak_unity_objects >= o.peak_unity_objects_min) &&
            (o.peak_unity_objects_max < 0 ||
             r.peak_unity_objects <= o.peak_unity_objects_max);
        std::snprintf(buf, sizeof(buf), "%d (oracle %s)",
                      r.peak_unity_objects, range);
        print_line("unity_peak:", buf, ok);
    }

    if (o.final_unity_objects_max >= 0) {
        std::snprintf(buf, sizeof(buf), "%d (oracle ≤ %d)",
                      r.final_unity_objects, o.final_unity_objects_max);
        print_line("unity_final:", buf,
                   r.final_unity_objects <= o.final_unity_objects_max);
    }

    std::printf("─────────────────────────────────────────────────────\n");
    std::printf("RESULT: %s%s\n\n", r.pass ? "PASS" : "FAIL  -- ",
                r.pass ? "" : r.fail_reason.c_str());
}

// ════════════════════════════════════════════════════════════════════
//   ZonePolicy 단위 테스트 (금지구역 침입 판정)
// ════════════════════════════════════════════════════════════════════
//   LiDAR 파이프라인과 독립인 순수 로직이라 합성 scene 없이 직접 검증한다.

static bool RunZonePolicyUnitTest() {
    bool ok = true;
    auto expect = [&ok](bool cond, const char* what) {
        if (!cond) {
            std::printf("  zone_policy: FAIL — %s\n", what);
            ok = false;
        }
    };

    ecowarden::ZonePolicyParams p;
    p.restricted_zones     = "1,3";
    p.intrusion_min_frames = 3;
    p.intrusion_repeat_ms  = 1000;
    p.forget_after_ms      = 500;

    ecowarden::ZonePolicy zp(p);
    ecowarden::IntrusionEvent evt;

    expect(zp.Enabled(), "restricted_zones 파싱 후 Enabled");
    expect(zp.IsRestricted(1) && zp.IsRestricted(3), "존 1/3 금지");
    expect(!zp.IsRestricted(0) && !zp.IsRestricted(2) && !zp.IsRestricted(4),
           "존 0/2/4 허용");
    expect(!zp.IsRestricted(-1) && !zp.IsRestricted(99), "범위 밖 존 허용 처리");

    // 일반 존에서는 이벤트가 없다.
    expect(!zp.OnPersonZone(7, 0, 100, 200, 1000, &evt), "일반 존 무이벤트");

    // 금지 존 연속 3프레임째에 확정된다.
    expect(!zp.OnPersonZone(7, 1, 100, 200, 1100, &evt), "1프레임 미확정");
    expect(!zp.OnPersonZone(7, 1, 100, 200, 1200, &evt), "2프레임 미확정");
    expect(zp.OnPersonZone(7, 1, 100, 200, 1300, &evt), "3프레임 확정");
    expect(evt.person_track_id == 7 && evt.zone == 1 &&
           evt.timestamp_ms == 1300, "이벤트 필드");

    // repeat 간격 이내에는 재알림하지 않는다.
    expect(!zp.OnPersonZone(7, 1, 100, 200, 1400, &evt), "재알림 억제");
    // repeat 간격이 지나면 다시 알린다.
    expect(zp.OnPersonZone(7, 1, 100, 200, 2400, &evt), "간격 경과 후 재알림");

    // 금지 존을 벗어나면 연속 카운트가 리셋된다.
    ecowarden::ZonePolicy zp2(p);
    zp2.OnPersonZone(8, 3, 0, 0, 100, &evt);
    zp2.OnPersonZone(8, 3, 0, 0, 200, &evt);
    zp2.OnPersonZone(8, 0, 0, 0, 300, &evt); // 이탈 → 리셋
    expect(!zp2.OnPersonZone(8, 3, 0, 0, 400, &evt), "이탈 후 카운트 리셋");

    // 정책 미설정 시 완전 비활성 (기존 동작 보존).
    ecowarden::ZonePolicyParams empty;
    ecowarden::ZonePolicy zpe(empty);
    expect(!zpe.Enabled(), "미설정 시 비활성");
    expect(!zpe.OnPersonZone(1, 1, 0, 0, 100, &evt), "비활성 시 무이벤트");

    // PruneStale: 오래 안 보인 사람 상태 제거.
    ecowarden::ZonePolicy zp3(p);
    zp3.OnPersonZone(9, 1, 0, 0, 100, &evt);
    zp3.PruneStale(100 + p.forget_after_ms + 1);
    expect(zp3.ActiveStates() == 0, "PruneStale 상태 정리");

    std::printf("─────────────────────────────────────────────────────\n");
    std::printf("SCENARIO: ZP_zone_policy_unit (unit test)\n");
    std::printf("─────────────────────────────────────────────────────\n");
    std::printf("RESULT: %s\n\n", ok ? "PASS" : "FAIL");
    return ok;
}

// ════════════════════════════════════════════════════════════════════
//   DumpValidator 단위 테스트 (투기 확정 후 재검증)
// ════════════════════════════════════════════════════════════════════
//   LiDAR 파이프라인과 독립인 순수 로직이라 합성 scene 없이 직접 검증한다.

static bool RunDumpValidationUnitTest() {
    bool ok = true;
    auto expect = [&ok](bool cond, const char* what) {
        if (!cond) {
            std::printf("  dump_validation: FAIL — %s\n", what);
            ok = false;
        }
    };

    ecowarden::DumpValidationParams p;
    p.enable             = true;
    p.validate_frames    = 5;
    p.max_move_mm        = 300.0;
    p.min_present_ratio  = 0.5;
    p.high_present_ratio = 0.8;
    p.source_departed_mm = 1200.0;

    auto make_evt = [](uint32_t obj_id) {
        ecowarden::DumpingEvent evt{};
        evt.person_track_id = 1;
        evt.person_x_mm     = 900.0;
        evt.person_y_mm     = 2000.0;
        evt.object_track_id = obj_id;
        evt.object_x_mm     = 1000.0;
        evt.object_y_mm     = 2000.0;
        evt.timestamp_ms    = 12345;
        return evt;
    };
    auto make_track = [](uint32_t id, double x, double y) {
        ecowarden::Track t{};
        t.id   = id;
        t.x_mm = x;
        t.y_mm = y;
        return t;
    };

    // Case A: 진짜 투기 — 투기물 잔존 + 투기자 이탈 → confidence=high.
    {
        ecowarden::DumpValidator dv(p);
        expect(dv.Enabled(), "enable=true 시 Enabled");
        dv.OnDumpConfirmed(make_evt(42), "img", 2, false, true, 100);
        expect(dv.PendingCount() == 1, "확정 후 pending 1건");
        // 같은 object 중복 확정은 무시된다.
        dv.OnDumpConfirmed(make_evt(42), "img", 2, false, true, 100);
        expect(dv.PendingCount() == 1, "중복 확정 무시");

        std::vector<ecowarden::ValidatedDump> out;
        for (uint32_t f = 101; f <= 105; f++) {
            std::vector<ecowarden::Track> tracks;
            tracks.push_back(make_track(42, 1000.0, 2000.0));     // 투기물 잔존
            tracks.push_back(make_track(1, 900.0 + f * 500.0,
                                        2000.0));                  // 투기자 이탈
            dv.Update(tracks, f, &out);
        }
        expect(out.size() == 1, "관찰 종료 후 검증 통과 1건");
        if (out.size() == 1) {
            expect(std::strcmp(out[0].confidence, "high") == 0,
                   "잔존+이탈 → confidence=high");
            expect(out[0].observed_frames == 5 && out[0].window_frames == 5,
                   "재관측 프레임 집계");
            expect(out[0].source_departed, "source 이탈 플래그");
            expect(out[0].evt.object_track_id == 42 &&
                   out[0].zone == 2 && out[0].pir_motion,
                   "원본 이벤트/컨텍스트 보존");
        }
        expect(dv.PendingCount() == 0, "통과 후 pending 비움");
    }

    // Case B: 고스트/반사 — 투기물이 사라짐 → 취소 (전송 없음).
    {
        ecowarden::DumpValidator dv(p);
        dv.OnDumpConfirmed(make_evt(43), "", 1, false, false, 200);
        std::vector<ecowarden::ValidatedDump> out;
        for (uint32_t f = 201; f <= 205; f++) {
            std::vector<ecowarden::Track> tracks;  // 투기물 트랙 없음
            dv.Update(tracks, f, &out);
        }
        expect(out.empty(), "고스트는 취소되어 전송 없음");
        expect(dv.PendingCount() == 0, "취소 후 pending 비움");
    }

    // Case C: 확정 후 이동 — 정지 투기물이 아님 → 즉시 취소.
    {
        ecowarden::DumpValidator dv(p);
        dv.OnDumpConfirmed(make_evt(44), "", 0, false, false, 300);
        std::vector<ecowarden::ValidatedDump> out;
        std::vector<ecowarden::Track> tracks;
        tracks.push_back(make_track(44, 1600.0, 2000.0));  // 600mm 이동
        dv.Update(tracks, 301, &out);
        expect(out.empty() && dv.PendingCount() == 0,
               "300mm 초과 이동 시 즉시 취소");
    }

    // Case D: 잔존은 충분하나 투기자가 근처에 남음 → confidence=medium.
    {
        ecowarden::DumpValidator dv(p);
        dv.OnDumpConfirmed(make_evt(45), "", 3, true, false, 400);
        std::vector<ecowarden::ValidatedDump> out;
        for (uint32_t f = 401; f <= 405; f++) {
            std::vector<ecowarden::Track> tracks;
            tracks.push_back(make_track(45, 1000.0, 2000.0));  // 잔존
            tracks.push_back(make_track(1, 900.0, 2000.0));    // 근처 잔류
            dv.Update(tracks, f, &out);
        }
        expect(out.size() == 1, "잔존 확인 시 통과");
        if (out.size() == 1) {
            expect(std::strcmp(out[0].confidence, "medium") == 0,
                   "source 잔류 → confidence=medium");
            expect(out[0].in_restricted, "금지구역 플래그 보존");
        }
    }

    // Case E: person_track_id 가 그룹 ID 인 경우도 이탈 판정된다 (v33).
    {
        ecowarden::DumpValidator dv(p);
        dv.OnDumpConfirmed(make_evt(46), "", 2, false, false, 500);
        std::vector<ecowarden::ValidatedDump> out;
        for (uint32_t f = 501; f <= 505; f++) {
            std::vector<ecowarden::Track> tracks;
            tracks.push_back(make_track(46, 1000.0, 2000.0));
            ecowarden::Track leg = make_track(77, 900.0, 2000.0);
            leg.person_group_id = 1;  // 그룹 ID = person_track_id
            tracks.push_back(leg);
            dv.Update(tracks, f, &out);
        }
        expect(out.size() == 1 &&
               std::strcmp(out[0].confidence, "medium") == 0,
               "그룹 트랙 근처 잔류 → medium (그룹 매칭 동작)");
    }

    std::printf("─────────────────────────────────────────────────────\n");
    std::printf("SCENARIO: DV_dump_validation_unit (unit test)\n");
    std::printf("─────────────────────────────────────────────────────\n");
    std::printf("RESULT: %s\n\n", ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace sim

// ════════════════════════════════════════════════════════════════════
//   main
// ════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    const char* only = nullptr;
    bool verbose = false;
    for (int i = 1; i < argc; i++) {
        if (std::strncmp(argv[i], "--only=", 7) == 0) only = argv[i] + 7;
        else if (std::strcmp(argv[i], "--verbose") == 0) verbose = true;
        else if (std::strcmp(argv[i], "-v") == 0) verbose = true;
    }

    std::printf("\n");
    std::printf("╔════════════════════════════════════════════════════╗\n");
    std::printf("║   sim regression suite (v55) — 30 tests             ║\n");
    std::printf("║   production_params.h 단일 출처 사용                 ║\n");
    std::printf("╚════════════════════════════════════════════════════╝\n\n");

    // v5 phase profiler (ECOWARDEN_PROFILE_CSV=... 설정 시 활성화)
    ecowarden::PhaseProfiler prof;
    if (prof.Enabled()) {
        std::printf("[PROF] phase profiler 활성 — CSV 기록 중\n\n");
    }

    int n_run  = 0;
    int n_pass = 0;
    int n_fail = 0;

    std::vector<sim::ScenarioResult> all;
    all.reserve(sim::kNumScenarios);

    for (size_t i = 0; i < sim::kNumScenarios && g_running; i++) {
        const auto& sc = sim::kScenarios[i];
        if (only && std::strstr(sc.name, only) == nullptr) continue;

        const auto r = sim::RunScenario(sc, verbose, &prof);
        sim::PrintScenarioResult(r);
        all.push_back(r);
        n_run++;
        if (r.pass) n_pass++;
        else        n_fail++;
    }

    // 금지구역(존) 정책 단위 테스트 — LiDAR scene 과 독립
    if (g_running &&
        (!only || std::strstr("ZP_zone_policy_unit", only) != nullptr)) {
        n_run++;
        if (sim::RunZonePolicyUnitTest()) {
            n_pass++;
        } else {
            n_fail++;
            std::printf("  - ZP_zone_policy_unit: unit assertions failed\n");
        }
    }

    // 투기 확정 재검증 단위 테스트 — LiDAR scene 과 독립
    if (g_running &&
        (!only || std::strstr("DV_dump_validation_unit", only) != nullptr)) {
        n_run++;
        if (sim::RunDumpValidationUnitTest()) {
            n_pass++;
        } else {
            n_fail++;
            std::printf("  - DV_dump_validation_unit: unit assertions failed\n");
        }
    }

    std::printf("═════════════════════════════════════════════════════\n");
    std::printf("SUMMARY: %d / %d PASS\n", n_pass, n_run);
    std::printf("═════════════════════════════════════════════════════\n");
    if (n_fail > 0) {
        std::printf("\nFailed scenarios:\n");
        for (const auto& r : all) {
            if (!r.pass) {
                std::printf("  - %s: %s\n", r.sc->name, r.fail_reason.c_str());
            }
        }
    }

    return n_fail;
}
