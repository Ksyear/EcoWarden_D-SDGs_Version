/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 메인 루프 (LiDAR + Camera + PIR)
 */

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <limits>

#include "background_filter.h"
#include "camera_module.h"
#include "cluster_tracker.h"
#include "dump_validation.h"
#include "event_notifier.h"
#include "json_packet.h"
#include "phase_profiler.h"
#include "production_params.h"
#include "rplidar_s2.h"
#include "scan_processor.h"
#include "servo_controller.h"
#include "servo_zone.h"
#include "time_utils.h"
#include "udp_sender.h"
#include "pir_sensor.h"
#include "zone_policy.h"

// ── Ctrl+C 처리 ─────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;
static void SignalHandler(int) { g_running = 0; }

// ── SIGUSR1: 배경 즉시 리셋 ─────────────────────────────────────────
//   반복 촬영/센서 재배치 시 프로세스 재시작 없이 배경만 다시 학습한다.
//   사용: kill -USR1 $(pgrep rplidar_app)
static volatile sig_atomic_t g_reset_bg = 0;
static void SigUsr1Handler(int) { g_reset_bg = 1; }

// ── "IP:PORT" 문자열 파싱 ───────────────────────────────────────────
static bool ParseIpPort(const char *str, std::string &ip, uint16_t &port) {
    const char *colon = std::strrchr(str, ':');
    if (!colon) return false;
    ip.assign(str, colon);
    port = static_cast<uint16_t>(std::atoi(colon + 1));
    return port > 0;
}

// ── 서보 조준/금지구역 판정 대상 사람 트랙 필터 ─────────────────────
//   zone 촬영과 intrusion 검사 양쪽에서 동일한 기준을 쓴다.
static bool IsPersonTargetTrack(const ecowarden::Track& tr) {
    if (tr.state == ecowarden::TrackState::Lost) return false;
    if (tr.lost_count > 0) return false;
    if (!tr.confirmed && tr.person_group_id == 0) return false;
    if (tr.is_dump_suspect || tr.is_dumped_item || tr.is_object_candidate) return false;
    if (ecowarden::IsBackgroundResidualTrack(tr)) return false;
    if (ecowarden::IsHeldPersonLeg(tr)) return false;
    if (tr.person_group_id != 0 && !tr.person_group_primary) return false;
    return true;
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    std::signal(SIGUSR1, SigUsr1Handler);

    // ── 인자 파싱 ────────────────────────────────────────────────────
    const char *serial_port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const char *api_url = (argc > 2) ? argv[2] : "https://api.ecowarden.systems/api/dumping-event";
    const char *unity_addr = (argc > 3) ? argv[3] : "192.168.20.102:5005";
    const char *viz_addr   = (argc > 4) ? argv[4] : "127.0.0.1:9090";

    // ── 1. LiDAR 및 프로세서 설정 ─────────────────────────────────────
    ecowarden::Config cfg;
    cfg.serial_port = serial_port;
    cfg.baudrate = 1000000;
    cfg.enable_filter = true;

    ecowarden::FilterParams           fp  = ecowarden::DefaultFilterParams();
    ecowarden::DBSCANParams           dp  = ecowarden::DefaultDBSCANParams();
    ecowarden::BackgroundFilterParams bgp = ecowarden::DefaultBackgroundFilterParams();
    ecowarden::TrackerParams          tp  = ecowarden::DefaultTrackerParams();

    ecowarden::ScanProcessor   processor(fp, dp);

    // ── 2. 배경 필터 및 추적기 설정 ──────────────────────────────────
    ecowarden::BackgroundFilter bg_filter(bgp);
    ecowarden::ClusterTracker  tracker(tp);

    // ── 2.9. 프라이버시 3계층 · 객체 분류 ───────────────────────────
    //
    //   1계층(평상시 무촬영)은 구조 자체로 보장되고, 여기서 2·3계층을 켠다.
    //     2계층 = 얼굴 마스킹 (기본 ON, fail-closed)
    //     3계층 = 원본 암호화 보관 (기본 OFF — 키를 넣고 명시적으로 켠다)
    //
    //   vault 는 camera 보다 먼저 선언해야 수명이 더 길다 (camera 가
    //   포인터만 들고 있으므로 역순 소멸 시 dangling 이 되면 안 된다).
    ecowarden::EvidenceVault evidence_vault(
        ecowarden::DefaultEvidenceVaultParams());
    if (!evidence_vault.Available() &&
        !evidence_vault.unavailable_reason().empty()) {
        std::printf("[VAULT] 원본 보관 비활성 — %s\n",
                    evidence_vault.unavailable_reason().c_str());
    }

    const ecowarden::ClassifierParams classifier_params =
        ecowarden::DefaultClassifierParams();
    ecowarden::ObjectClassifier classifier(classifier_params);
    std::printf("[CLASSIFY] 객체 분류 백엔드: %s\n", classifier.BackendName());

    // ── 3. 하드웨어 모듈 시작 ────────────────────────────────────────
    ecowarden::CameraModule camera(0);
    camera.SetCaptureDir("captures");
    const ecowarden::BlackboxParams blackbox_params =
        ecowarden::DefaultBlackboxParams();
    camera.ConfigureBlackbox(blackbox_params);
    camera.ConfigureHeadLabel(ecowarden::DefaultHeadLabelParams());

    const ecowarden::FaceMaskParams face_mask_params =
        ecowarden::DefaultFaceMaskParams();
    camera.ConfigureFaceMask(face_mask_params);
    camera.SetEvidenceVault(evidence_vault.Available() ? &evidence_vault
                                                       : nullptr);

    if (!camera.Start()) {
        std::fprintf(stderr, "[ERROR] Camera Start Failed!\n");
    } else {
        if (blackbox_params.enable) {
            std::printf("[BLACKBOX] 전 %u초 + 후 %u초 @ %.1ffps 링버퍼 활성\n",
                        blackbox_params.pre_seconds,
                        blackbox_params.post_seconds, blackbox_params.fps);
        }
        if (face_mask_params.enable) {
            std::printf("[FACEMASK] 얼굴 마스킹 활성 — 백엔드=%s 방식=%s%s\n",
                        camera.FaceMaskBackend(),
                        ecowarden::FaceMaskModeToString(face_mask_params.mode),
                        face_mask_params.require_detector ? " (fail-closed)"
                                                          : "");
        } else {
            std::printf("[FACEMASK] ⚠ 얼굴 마스킹 비활성 "
                        "(ECOWARDEN_FACE_MASK=1 권장)\n");
        }
    }

    ecowarden::ServoParams servo_params = ecowarden::DefaultServoParams();
    ecowarden::ServoController servo(servo_params, camera);
    if (!servo.Init()) {
        std::fprintf(stderr,
                     "[WARN] Servo unavailable; suspect capture uses fixed camera fallback\n");
    }

    // ── 3.5. 금지구역(존) 정책 ──────────────────────────────────────
    //   ECOWARDEN_RESTRICTED_ZONES="3,4" 형식. 비어 있으면 비활성.
    const ecowarden::ZonePolicyParams zone_policy_params =
        ecowarden::DefaultZonePolicyParams();
    ecowarden::ZonePolicy zone_policy(zone_policy_params);
    if (zone_policy.Enabled()) {
        std::printf("[ZONE] 금지구역 활성: zones=%s (연속 %u프레임, 재알림 %ums)\n",
                    zone_policy_params.restricted_zones.c_str(),
                    zone_policy_params.intrusion_min_frames,
                    zone_policy_params.intrusion_repeat_ms);
    }

    // ── 3.6. 투기 확정 재검증 (오탐 차단) ───────────────────────────
    //   확정된 투기물이 실제로 현장에 남아 있는지 N프레임 재관찰 후에만
    //   서버 전송/Unity 표시. 고스트·반사·잔상 오탐을 전송 전에 걸러낸다.
    //   ECOWARDEN_DUMP_VALIDATE=0 으로 끄면 기존처럼 확정 즉시 전송.
    const ecowarden::DumpValidationParams dump_validation_params =
        ecowarden::DefaultDumpValidationParams();
    ecowarden::DumpValidator dump_validator(dump_validation_params);
    if (dump_validator.Enabled()) {
        std::printf("[DUMP-VERIFY] 확정 후 재검증 활성: %u프레임 잔존 확인"
                    " (이동 %.0fmm 초과 또는 재관측 %.0f%% 미만이면 취소)\n",
                    dump_validation_params.validate_frames,
                    dump_validation_params.max_move_mm,
                    dump_validation_params.min_present_ratio * 100.0);
    }

    // ── 4. 출력 및 네트워크 설정 ─────────────────────────────────────
    ecowarden::NotifierConfig nc;
    nc.endpoint_url = api_url;
    if (const char* env = std::getenv("ECOWARDEN_DRY_RUN")) {
        if (env[0] == '1' || env[0] == 't' || env[0] == 'T') {
            nc.dry_run = true;
        }
    }
    ecowarden::EventNotifier notifier(nc);
    notifier.StartRetryThread();

    // 블랙박스 클립 저장이 끝나면 서버로 비동기 업로드한다 (사진은 이벤트
    // 시점에 base64 로 즉시 전송되고, 영상은 완성 후 multipart 로 따라간다).
    camera.SetBlackboxSavedCallback(
        [&notifier](const std::string& path,
                    const ecowarden::BlackboxClipMeta& meta) {
            notifier.UploadClip(path, meta);
        });

    // 투기 의심(Suspect) 시점에는 scan loop를 막지 않고 서보 worker에 위임한다.
    tracker.SetSuspectCallback([&servo](const ecowarden::DumpingSuspectEvent& evt) {
        servo.OnSuspect(evt);
    });

    // Unity UDP 전송 설정
    ecowarden::UdpSenderConfig uc;
    std::string u_ip; uint16_t u_port;
    if (ParseIpPort(unity_addr, u_ip, u_port)) {
        uc.dest_ip = u_ip;
        uc.dest_port = u_port;
    }

    ecowarden::UdpSender udp(uc);
    if (!udp.Open()) {
        std::fprintf(stderr, "[WARN] Unity UDP Open Failed\n");
    }
    ecowarden::JsonPacketSender json_sender(udp);

    // 시각화기 전용 UDP 전송 설정 (localhost:9090)
    ecowarden::UdpSenderConfig vc;
    std::string v_ip; uint16_t v_port;
    if (ParseIpPort(viz_addr, v_ip, v_port)) {
        vc.dest_ip = v_ip;
        vc.dest_port = v_port;
    }

    ecowarden::UdpSender viz_udp(vc);
    if (!viz_udp.Open()) {
        std::fprintf(stderr, "[WARN] Visualizer UDP Open Failed\n");
    }
    ecowarden::JsonPacketSender viz_sender(viz_udp);

    // ── 5. 센서 시작 ────────────────────────────────────────────────
    // PIR 센서 (2개: 좌측 GPIO 17, 우측 GPIO 27)
    ecowarden::PirSensor pir_left({17, true, 2, 30});
    ecowarden::PirSensor pir_right({27, true, 2, 30});
    pir_left.Init();
    pir_right.Init();

    // LiDAR 시작
    ecowarden::RplidarS2 lidar;
    if (lidar.Start(cfg) != ecowarden::Error::None) {
        std::fprintf(stderr, "[ERROR] LiDAR Start Failed\n");
        return 1;
    }

    std::printf("=== EcoWarden 불법 투기 감지 시스템 (LiDAR Illegal Dumping Detection) Started ===\n");
    std::printf("  Unity  : %s:%u\n", uc.dest_ip.c_str(), uc.dest_port);
    std::printf("  Viz    : %s:%u\n", vc.dest_ip.c_str(), vc.dest_port);
    std::printf("  FastAPI: %s\n", api_url);

    bool frame_logging = false;
    if (const char* env = std::getenv("ECOWARDEN_LOG_LEVEL")) {
        if (std::strcmp(env, "frame") == 0 || std::strcmp(env, "debug") == 0) {
            frame_logging = true;
            std::printf("[LOG] 프레임 로깅 활성화 (ECOWARDEN_LOG_LEVEL=%s)\n", env);
        }
    }

    bool send_unity_binary = false;
    if (const char* env = std::getenv("ECOWARDEN_UNITY_BINARY")) {
        send_unity_binary = (env[0] == '1' || env[0] == 't' || env[0] == 'T');
    }

    ecowarden::PhaseProfiler prof;
    if (prof.Enabled()) {
        std::printf("[PROF] phase profiler active\n");
    }

    auto MaxKfUncertainty = [](const std::vector<ecowarden::Track>& tracks) {
        double m = 0.0;
        for (const auto& t : tracks) {
            if (t.kf.IsInitialized()) {
                double u = t.kf.PositionUncertaintySq();
                if (u > m) m = u;
            }
        }
        return m;
    };

    uint32_t active_person_target_id = 0;

    auto RequestPersonZoneCapture =
        [&servo, &active_person_target_id](const std::vector<ecowarden::Track>& tracks) {
            const ecowarden::Track* best = nullptr;
            if (active_person_target_id != 0) {
                for (const auto& tr : tracks) {
                    if (!IsPersonTargetTrack(tr)) continue;
                    const uint32_t id = tr.person_group_id ? tr.person_group_id : tr.id;
                    if (id == active_person_target_id) {
                        best = &tr;
                        break;
                    }
                }
            }

            if (!best) {
                active_person_target_id = 0;
                double best_d2 = std::numeric_limits<double>::max();
                for (const auto& tr : tracks) {
                    if (!IsPersonTargetTrack(tr)) continue;

                    const double x = tr.person_group_id ? tr.person_group_x_mm : tr.x_mm;
                    const double y = tr.person_group_id ? tr.person_group_y_mm : tr.y_mm;
                    const double d2 = x * x + y * y;
                    if (d2 < best_d2) {
                        best = &tr;
                        best_d2 = d2;
                    }
                }
            }

            if (best) {
                const uint32_t id = best->person_group_id ? best->person_group_id : best->id;
                const double x = best->person_group_id ? best->person_group_x_mm : best->x_mm;
                const double y = best->person_group_id ? best->person_group_y_mm : best->y_mm;
                active_person_target_id = id;
                servo.OnPersonDetected(id, x, y);
            }
        };

    // ── 6. 메인 루프 ────────────────────────────────────────────────
    uint32_t frame_count = 0;
    uint32_t scan_fail_count = 0;
    constexpr uint32_t kScanFailReconnectThreshold = 5;
    while (g_running) {
        prof.BeginFrame();

        // SIGUSR1: 배경 즉시 리셋 (반복 촬영/센서 재배치용)
        if (g_reset_bg) {
            g_reset_bg = 0;
            processor.ResetStaticBackground();
            bg_filter.Reset();
            std::fprintf(stderr, "[RESET] 배경 학습 재시작 (%u 프레임)\n",
                         bgp.learning_frames);
        }

        // ── 보존기한 만료 원본 자동 파기 (프라이버시 3계층) ──────────
        //   10Hz 기준 36,000 프레임 ≈ 1시간마다 한 번 훑는다. 파일 헤더만
        //   읽고 판정하므로 복호화 비용은 없다.
        if (evidence_vault.Available() && frame_count > 0 &&
            frame_count % 36000 == 0) {
            evidence_vault.SweepExpired();
        }

        ecowarden::ScanFrame frame;
        if (lidar.GetScanFrame(frame) != ecowarden::Error::None) {
            // USB 분리/케이블 불량 등으로 스캔이 연속 실패하면 재연결한다.
            // 배경 맵은 유지해 재연결 후 재학습 대기를 만들지 않는다.
            if (++scan_fail_count >= kScanFailReconnectThreshold) {
                std::fprintf(stderr,
                             "[LIDAR] 스캔 %u회 연속 실패 — 재연결 시도\n",
                             scan_fail_count);
                lidar.Stop();
                while (g_running) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    if (lidar.Start(cfg) == ecowarden::Error::None) {
                        std::fprintf(stderr, "[LIDAR] 재연결 성공\n");
                        break;
                    }
                    std::fprintf(stderr, "[LIDAR] 재연결 실패 — 2초 후 재시도\n");
                }
                scan_fail_count = 0;
            }
            continue;
        }
        scan_fail_count = 0;
        if (frame.empty()) continue;
        prof.Mark(ecowarden::PhaseProfiler::kScan);

        frame_count++;

        // (1) 필터 및 클러스터링 (정적 배경 학습 전에 처리 → 첫 프레임에서도 클러스터 생성)
        std::vector<ecowarden::Cluster> clusters;
        processor.Process(frame, clusters);
        prof.Mark(ecowarden::PhaseProfiler::kFilter);

        // (1.5) 정적 배경 깊이 맵 학습
        //   추적 중인 객체만 학습 차단한다. 모든 클러스터를 막으면 기존 배경/정적 물체가
        //   배경으로 재흡수되지 못하고 사람 후보 트랙으로 승격될 수 있다.
        {
            bool blocked[3600] = {};
            for (const auto& tr : tracker.GetTracks()) {
                if (tr.lost_count > 5) continue;
                if (ecowarden::IsBackgroundResidualTrack(tr)) continue;
                // 트랙 centroid → 극좌표 각도 → 0.1도 슬롯
                double ang = std::atan2(tr.y_mm, tr.x_mm) * 180.0 / M_PI;
                if (ang < 0.0) ang += 360.0;
                int center = static_cast<int>(std::round(ang * 10.0)) % 3600;
                // ±3도(30슬롯) 범위 차단 — 사람 폭 고려
                for (int d = -30; d <= 30; ++d) {
                    int s = (center + d + 3600) % 3600;
                    blocked[s] = true;
                }
            }
            processor.LearnStaticBackground(frame, blocked);
        }

        // PIR은 배경 학습 중에도 계속 읽어 holdoff/debounce 상태를 유지한다.
        pir_left.Read();
        pir_right.Read();

        // (2) 배경 필터링 (동적 객체 추출)
        if (!bg_filter.Apply(clusters)) {
            if (bg_filter.GetFrameCount() == 1) {
                std::printf("[INFO] 배경 학습 시작 (%u 프레임)...\n", bgp.learning_frames);
            }
            prof.Mark(ecowarden::PhaseProfiler::kBackground);

            // 학습 중에도 시각화 데이터 전송 (점구름 + 빈 트랙/이벤트)
            std::vector<ecowarden::DepartureEvent> empty_dep;
            std::vector<ecowarden::DumpingEvent> empty_dump;
            std::vector<ecowarden::Track> empty_tracks;
            json_sender.Send(clusters, empty_tracks, empty_dep, empty_dump, frame_count);
            viz_sender.Send(clusters, empty_tracks, empty_dep, empty_dump, frame_count);

            prof.EndFrame(MaxKfUncertainty(tracker.GetTracks()));
            continue;
        }
        prof.Mark(ecowarden::PhaseProfiler::kBackground);

        // (3.5) 배경 필터 보호 위치 피드백
        {
            std::vector<std::pair<double,double>> protected_positions;
            for (const auto& tr : tracker.GetTracks()) {
                if (tr.lost_count > 5) continue;
                if (ecowarden::IsBackgroundResidualTrack(tr)) continue;

                if (tr.is_dumped_item || tr.is_dump_suspect || tr.is_object_candidate) {
                    protected_positions.push_back({tr.x_mm, tr.y_mm});
                    continue;
                }

                if (tr.person_group_id != 0) {
                    if (tr.person_group_primary) {
                        protected_positions.push_back({tr.person_group_x_mm, tr.person_group_y_mm});
                    }
                    continue;
                }

                if (tr.confirmed) {
                    protected_positions.push_back({tr.x_mm, tr.y_mm});
                }
            }
            bg_filter.SetProtectedForegroundPositions(protected_positions);
        }

        // (4) 객체 추적 및 이벤트 감지
        std::vector<ecowarden::DepartureEvent> dep_events;
        std::vector<ecowarden::DumpingEvent> dump_events;
        std::vector<ecowarden::IntrusionEvent> intrusion_events;
        tracker.Update(clusters, dep_events, dump_events);
        prof.Mark(ecowarden::PhaseProfiler::kTracker);

        // 사람/사람그룹이 감지된 zone을 주기적으로 조준 촬영한다.
        // 투기 suspect job이 있으면 ServoController 내부에서 suspect 촬영을 우선한다.
        RequestPersonZoneCapture(tracker.GetTracks());
        servo.CleanupPersonSessions();

        // (4.5) 금지구역 침입 검사 — 금지 존에 사람이 연속 관측되면
        //   카메라를 조준하고 intrusion 이벤트 + 블랙박스 영상을 남긴다.
        if (zone_policy.Enabled()) {
            const uint64_t now_ms = ecowarden::NowMs();
            for (const auto& tr : tracker.GetTracks()) {
                if (!IsPersonTargetTrack(tr)) continue;
                const uint32_t pid = tr.person_group_id ? tr.person_group_id : tr.id;
                const double px = tr.person_group_id ? tr.person_group_x_mm : tr.x_mm;
                const double py = tr.person_group_id ? tr.person_group_y_mm : tr.y_mm;
                const int zone = ecowarden::SuspectToZone(px, py, servo_params.mirror);

                ecowarden::IntrusionEvent ievt;
                if (!zone_policy.OnPersonZone(pid, zone, px, py, now_ms, &ievt)) {
                    continue;
                }

                // ── 객체 분류 → 이벤트 등급 (전체_구조.md §6) ────────
                //   LiDAR 기하 특징으로 사람/동물/불명을 가르고 등급을
                //   매긴다. noise 로 분류되면 이벤트 자체를 올리지 않아
                //   반사·스파이크로 인한 헛알림을 줄인다.
                ecowarden::LidarFeatures lf;
                lf.width_mm = tr.person_group_id ? tr.person_group_width_mm
                                                 : tr.width_mm;
                lf.speed_mm_s =
                    std::hypot(tr.x_mm - tr.prev_x_mm, tr.y_mm - tr.prev_y_mm) *
                    10.0;   // 10Hz → mm/s
                lf.age_frames        = tr.age;
                lf.match_frames      = tr.total_match_count;
                lf.stationary_frames = tr.stationary_count;
                lf.in_person_group   = tr.person_group_id != 0;
                lf.was_moving        = tr.was_moving;

                const ecowarden::ClassifyResult cr =
                    classifier.ClassifyFromLidar(lf);
                const ecowarden::EventSeverity sev =
                    ecowarden::GradeEvent(cr, /*restricted_zone=*/true);

                if (sev == ecowarden::EventSeverity::kIgnore) {
                    std::printf("[CLASSIFY] zone %d 이벤트 무시 — %s (%s)\n",
                                zone, ecowarden::ObjectClassToString(cr.cls),
                                cr.reason.c_str());
                    continue;
                }

                ievt.object_class = ecowarden::ObjectClassToString(cr.cls);
                ievt.severity     = ecowarden::EventSeverityToString(sev);
                ievt.event_type   = ecowarden::EventTypeForClass(cr.cls);
                ievt.classify_confidence = cr.confidence;

                std::printf("\a[!!] RESTRICTED ZONE INTRUSION: person %u in zone %d at (%.0f, %.0f)"
                            " — %s/%s (%s, conf=%.2f)\n",
                            pid, zone, px, py, ievt.event_type, ievt.severity,
                            cr.reason.c_str(), cr.confidence);
                servo.OnPersonDetected(pid, px, py); // 카메라 즉시 조준
                notifier.SendIntrusion(ievt, camera.CaptureBase64());
                ecowarden::BlackboxClipMeta clip_meta;
                clip_meta.event_type = "intrusion";
                clip_meta.person_id = pid;
                clip_meta.zone = ievt.zone;
                clip_meta.event_time_ns =
                    static_cast<int64_t>(ievt.timestamp_ms) * 1000000LL;
                camera.RequestBlackboxSave(
                    "captures/intrusions/intrusion_" + std::to_string(pid) +
                    "_" + std::to_string(now_ms) + ".avi", clip_meta);
                intrusion_events.push_back(ievt); // Unity/시각화기 침입 경보 표시용
            }
            zone_policy.PruneStale(now_ms);
        }

        // (5) 이벤트 처리 (투기 확정 시 증거 사진 확보 및 전송)
        //   촬영 후 `image_file`(captures/ 기준 상대 경로)을 채운 사본을
        //   만들어 이후 경로(재검증/서버/Unity)에 일관되게 흘려보낸다 (v56).
        std::vector<ecowarden::DumpingEvent> enriched_dumps;
        enriched_dumps.reserve(dump_events.size());
        for (const auto& evt_in : dump_events) {
            ecowarden::DumpingEvent evt = evt_in;   // 가변 복사본
            const int dump_zone = ecowarden::SuspectToZone(
                evt.object_x_mm, evt.object_y_mm, servo_params.mirror);
            const bool in_restricted = zone_policy.IsRestricted(dump_zone);

            std::printf("\a[!!!] DUMPING DETECTED at (%.0f, %.0f) zone=%d%s\n",
                        evt.object_x_mm, evt.object_y_mm, dump_zone,
                        in_restricted ? " [RESTRICTED ZONE]" : "");

            bool motion = pir_left.IsMotionDetected() || pir_right.IsMotionDetected();

            std::string img_path;
            std::string blackbox_path;
            auto bundle = servo.PromoteSuspectToDump(evt.object_track_id,
                                                     evt.person_track_id);
            if (bundle) {
                img_path = bundle->confirm_path;
                // 블랙박스 클립은 dump bundle 디렉터리에 함께 보관
                const std::string meta = bundle->meta_path;
                const size_t slash = meta.find_last_of('/');
                blackbox_path = (slash != std::string::npos)
                    ? meta.substr(0, slash) + "/blackbox.avi"
                    : "captures/dumps/blackbox_" +
                          std::to_string(evt.timestamp_ms) + ".avi";
            } else {
                img_path = camera.CaptureToFile();
                blackbox_path = "captures/dumps/blackbox_" +
                    std::to_string(evt.timestamp_ms) + ".avi";
            }
            ecowarden::BlackboxClipMeta clip_meta;
            clip_meta.event_type = "dumping";
            clip_meta.person_id = evt.person_track_id;
            clip_meta.object_id = evt.object_track_id;
            clip_meta.zone = dump_zone;
            clip_meta.event_time_ns =
                static_cast<int64_t>(evt.timestamp_ms) * 1000000LL;
            camera.RequestBlackboxSave(blackbox_path, clip_meta);

            std::string img_b64 = camera.CaptureBase64();

            // Unity 가 정확한 증거 사진을 집을 수 있도록 상대 경로를 싣는다.
            //   img_path 는 "captures/dumps/evt_1/confirm.jpg" 형태 →
            //   앞의 "captures/" 를 떼어 "dumps/evt_1/confirm.jpg" 로 보낸다.
            //   경로를 못 얻으면 필드를 비워 두고, Unity 는 기존처럼
            //   "최신 사진" 으로 폴백한다.
            if (!img_path.empty()) {
                constexpr const char* kPrefix = "captures/";
                evt.image_file = (img_path.rfind(kPrefix, 0) == 0)
                    ? img_path.substr(std::strlen(kPrefix))
                    : img_path;
            }
            enriched_dumps.push_back(evt);

            std::printf("  - Photo: %s (PIR: %s)\n", img_path.empty() ? "FAILED" : img_path.c_str(), motion ? "YES" : "NO");

            if (dump_validator.Enabled()) {
                // 증거(서보 번들/사진/블랙박스)는 위에서 이미 확보했다.
                // 서버 전송과 Unity 표시만 재검증 통과 후로 미룬다 — (5.5).
                dump_validator.OnDumpConfirmed(evt, img_b64, dump_zone,
                                               in_restricted, motion,
                                               frame_count);
                std::printf("  - [DUMP-VERIFY] object %u 재검증 시작 —"
                            " %u프레임 잔존 확인 후 전송\n",
                            evt.object_track_id,
                            dump_validation_params.validate_frames);
            } else {
                notifier.SendDumping(evt, img_b64, dump_zone, in_restricted);
            }
        }

        for (const auto& evt : dep_events) {
            notifier.Send(evt);
        }

        // (5.5) 투기 확정 재검증 — 투기물이 실제 잔존한 확정만 서버 전송 +
        //   Unity `dumping` 이벤트로 내보낸다. 고스트/반사는 [DUMP-CANCEL]
        //   로그만 남고 전송되지 않는다 (로컬 증거 파일은 유지).
        std::vector<ecowarden::DumpingEvent> outgoing_dump_events;
        if (dump_validator.Enabled()) {
            std::vector<ecowarden::ValidatedDump> validated;
            dump_validator.Update(tracker.GetTracks(), frame_count, &validated);
            for (const auto& v : validated) {
                notifier.SendDumping(v.evt, v.image_base64, v.zone,
                                     v.in_restricted, v.confidence,
                                     static_cast<int>(v.observed_frames),
                                     static_cast<int>(v.window_frames));
                outgoing_dump_events.push_back(v.evt);
            }
        } else {
            outgoing_dump_events = enriched_dumps;   // image_file 포함본
        }

        prof.Mark(ecowarden::PhaseProfiler::kEvents);

        // (6) Unity + 시각화기 데이터 전송 (JSON)
        //   Unity: tracked_only=true → 추적 확인된 객체만 (고스트/노이즈 제거)
        //   시각화기: tracked_only=false → 전체 클러스터 + 점구름 (디버그용)
        //   dumping 이벤트는 재검증 통과분(outgoing_dump_events)만 내보낸다.
        json_sender.Send(clusters, tracker.GetTracks(), dep_events, outgoing_dump_events, frame_count, true, intrusion_events);
        viz_sender.Send(clusters, tracker.GetTracks(), dep_events, outgoing_dump_events, frame_count, false, intrusion_events);

        // (6.5) Unity 바이너리 프로토콜 전송
        //   기본 Unity 수신기는 JSON 전용이므로 바이너리는 명시적으로 켰을 때만 보낸다.
        if (send_unity_binary) {
            const auto& all_tracks = tracker.GetTracks();
            std::vector<ecowarden::Cluster> tracked_clusters;
            std::vector<ecowarden::CartesianPoint> tracked_points;
            tracked_clusters.reserve(all_tracks.size());

            for (const auto& tr : all_tracks) {
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

                ecowarden::Cluster out{};
                out.centroid_x_mm = tr.person_group_id ? tr.person_group_x_mm : tr.x_mm;
                out.centroid_y_mm = tr.person_group_id ? tr.person_group_y_mm : tr.y_mm;
                out.width_mm      = tr.person_group_id ? tr.person_group_width_mm : tr.width_mm;
                tracked_clusters.push_back(std::move(out));
            }
            udp.SendPoints(tracked_points, frame_count);
            udp.SendClusters(tracked_clusters, all_tracks, frame_count);
        }
        prof.Mark(ecowarden::PhaseProfiler::kJsonUdp);

        double kf_unc = MaxKfUncertainty(tracker.GetTracks());
        prof.EndFrame(kf_unc);
        prof.PrintRollingSummary(100);

        if (frame_logging) {
            int active = 0, suspects = 0, dumped = 0;
            for (const auto& tr : tracker.GetTracks()) {
                if (tr.state != ecowarden::TrackState::Lost) active++;
                if (tr.is_dump_suspect) suspects++;
                if (tr.is_dumped_item) dumped++;
            }
            std::printf("[F#%u] t=%d c=%zu d=%zu s=%d dmp=%d kf=%.1f bg=%zu\n",
                        frame_count, active, clusters.size(),
                        dump_events.size(), suspects, dumped, kf_unc,
                        bg_filter.GetBackgroundCount());
        }
    }

    // ── 7. 정리 ─────────────────────────────────────────────────────
    std::printf("\nStopping system...\n");
    lidar.Stop();
    servo.Shutdown();
    camera.Stop();
    notifier.StopRetryThread();
    udp.Close();
    viz_udp.Close();

    return 0;
}
