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
 * @file  event_notifier.h
 * @brief 이탈/투기 이벤트를 FastAPI 서버로 HTTP POST 전송 (비동기) + 실패 시 로컬 파일 큐잉
 * @date  2026
 *
 * 흐름:
 *   Send() / SendDumping()
 *       │
 *       ▼
 *   send_queue_ (비동기 큐)
 *       │
 *       ▼ (전송 스레드)
 *   HttpPost()
 *       │
 *       ├─ 성공 → 완료
 *       └─ 실패 → pending_queue_ + 로컬 파일
 *                  │
 *                  ▼ (재전송 스레드)
 *              FlushQueue()
 */

#pragma once

#include <string>
#include <utility>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "camera_module.h"
#include "cluster_tracker.h"
#include "zone_policy.h"

namespace ecowarden {

// ── 전송 설정 ────────────────────────────────────────────────────────
struct NotifierConfig {
    std::string endpoint_url    = "https://api.ecowarden.systems/api/dumping-event";
    // 금지구역 침입 이벤트 endpoint. 비어 있으면 endpoint_url 의
    // "dumping-event" 를 "intrusion-event" 로 치환해 자동 유도하고,
    // 치환 대상이 없으면 endpoint_url 을 그대로 쓴다.
    // ECOWARDEN_INTRUSION_URL 환경변수로 오버라이드 가능.
    std::string intrusion_url   = "";
    // 서버가 intrusion endpoint 를 아직 지원하지 않으면
    // ECOWARDEN_INTRUSION_NOTIFY=0 으로 전송만 끈다 (로그/촬영은 유지).
    bool        intrusion_notify = true;
    // 증거 영상(블랙박스 클립) 업로드 endpoint. 비어 있으면 endpoint_url 의
    // "dumping-event" 를 "evidence-clip" 으로 치환해 자동 유도.
    // ECOWARDEN_CLIP_URL 환경변수로 오버라이드 가능.
    std::string clip_url        = "";
    // 서버가 클립 업로드를 아직 지원하지 않으면 ECOWARDEN_CLIP_UPLOAD=0
    // 으로 업로드만 끈다 (클립 파일은 기기에 계속 저장됨).
    bool        clip_upload     = true;
    // 클립(수십 MB) 전용 업로드 타임아웃 — 일반 이벤트 3초와 분리.
    long        clip_timeout_ms = 120000;
    uint32_t    clip_max_retries = 3;
    // X-API-Key 값. 비어 있으면 생성자가 ECOWARDEN_API_KEY 환경변수에서 읽는다.
    // 소스에 키를 하드코딩하지 말 것 — 배포 시 systemd EnvironmentFile 로 주입.
    std::string api_key         = "";
    std::string queue_file_path = "/tmp/rplidar_event_queue.jsonl";

    // 큐에 남은 이벤트의 최대 나이(초). 이보다 오래된 것은 재전송하지 않고
    // 버린다. 0 이면 무제한(기존 동작).
    //
    //   재시작 시 어제 이벤트가 한꺼번에 쏟아지는 것을 막는다. 시연 중
    //   파이가 재시작되면 수십~수백 건의 과거 투기가 "방금 일어난 것"처럼
    //   대시보드에 뜨는데, 증거 시각이 과거라 오히려 혼란만 준다.
    long        queue_max_age_sec = 3600;   // 1시간
    long        timeout_ms      = 3000;        // HTTP 타임아웃 (ms)
    uint32_t    max_retries     = 3;           // 큐 재전송 시 최대 재시도 횟수
    uint32_t    retry_interval_sec = 10;       // 큐 재전송 주기 (초)

    // HTTP 전송을 완전히 끄고 싶을 때 사용 (모든 이벤트 drop, 성공 반환 안 함).
    //   - true:  HttpPost가 네트워크 호출 없이 즉시 false를 반환하고 dropped_count_만 증가.
    //            pending_queue 에 쌓는 대신 완전히 폐기 (queue_file_path 도 쓰지 않음).
    //   - false: 정상 HTTP POST (기본값).
    // 기존 "사용자 요청으로 주석 처리"의 역할을 config로 끌어올린 것.
    bool        dry_run         = false;
};

// ── 이벤트 전송기 ────────────────────────────────────────────────────
class EventNotifier {
public:
    explicit EventNotifier(const NotifierConfig& cfg = NotifierConfig{});
    ~EventNotifier();

    EventNotifier(const EventNotifier&) = delete;
    EventNotifier& operator=(const EventNotifier&) = delete;

    /**
     * @brief 이탈 이벤트를 비동기 전송 큐에 넣는다 (논블로킹).
     */
    void Send(const DepartureEvent& evt);

    /**
     * @brief 투기 이벤트를 비동기 전송 큐에 넣는다 (논블로킹).
     * @param image_base64 (선택) 카메라 모듈이 캡처한 Base64 인코딩 이미지 문자열
     * @param zone (선택) 투기 발생 존(0~4). 음수면 페이로드에서 생략
     * @param restricted_zone true 면 금지구역 내 투기 → severity="high"
     * @param confidence (선택) 확정 후 재검증 결과("high"/"medium").
     *        비어 있으면 confidence 관련 필드는 생략 (레거시 페이로드 동일)
     * @param validated_frames (선택) 재검증 창에서 투기물이 재관측된 프레임 수
     * @param validate_window (선택) 재검증 창 길이 (프레임)
     */
    void SendDumping(const DumpingEvent& evt, const std::string& image_base64 = "",
                     int zone = -1, bool restricted_zone = false,
                     const std::string& confidence = "",
                     int validated_frames = -1, int validate_window = -1);

    /**
     * @brief 금지구역 침입 이벤트를 비동기 전송 큐에 넣는다 (논블로킹).
     *        cfg.intrusion_notify=false 면 전송하지 않고 로그만 남긴다.
     */
    void SendIntrusion(const IntrusionEvent& evt, const std::string& image_base64 = "");

    /**
     * @brief 증거 영상 클립을 비동기 업로드 큐에 넣는다 (논블로킹).
     *        multipart/form-data 로 clip_url 에 업로드하며, 실패해도
     *        클립 파일은 기기에 남는다. cfg.clip_upload=false 면 로그만.
     *        CameraModule::SetBlackboxSavedCallback 과 연결해 사용한다.
     */
    void UploadClip(const std::string& path, const BlackboxClipMeta& meta);

    /**
     * @brief 큐 파일에 쌓인 이벤트를 모두 재전송한다.
     * @return 성공적으로 재전송된 이벤트 수
     */
    size_t FlushQueue();

    /**
     * @brief 전송 스레드 + 재전송 스레드를 시작한다.
     */
    void StartRetryThread();

    /**
     * @brief 모든 스레드를 중지한다.
     */
    void StopRetryThread();

    size_t GetQueueSize() const;
    size_t GetSentCount() const    { return sent_count_; }
    size_t GetFailCount() const    { return fail_count_; }
    size_t GetDroppedCount() const { return dropped_count_; }
    bool   IsDryRun() const        { return cfg_.dry_run; }

private:
    NotifierConfig cfg_;

    std::atomic<size_t> sent_count_{0};
    std::atomic<size_t> fail_count_{0};
    // dry_run 모드 또는 백엔드 영구 실패로 drop된 이벤트 수.
    // 기존엔 주석 처리된 HttpPost가 성공처럼 sent_count_를 증가시키는 바람에
    // "실제 전송된 것 / drop된 것"을 구분할 수 없었다.
    std::atomic<size_t> dropped_count_{0};

    // ── 비동기 전송 큐 (Send 호출 → 전송 스레드가 처리) ────────────
    mutable std::mutex          send_queue_mutex_;
    std::queue<std::string>     send_queue_;
    std::condition_variable     send_queue_cv_;
    std::thread                 send_thread_;
    std::atomic<bool>           send_running_{false};

    // ── 실패 이벤트 큐 (전송 실패 → 파일+메모리 큐 → 재전송 스레드) ─
    mutable std::mutex          fail_queue_mutex_;
    std::queue<std::string>     pending_queue_;

    // ── 백그라운드 재전송 스레드 ────────────────────────────────────
    std::thread                 retry_thread_;
    std::atomic<bool>           retry_running_{false};
    std::mutex                  retry_cv_mutex_;
    std::condition_variable     retry_cv_;

    // ── 증거 영상 클립 업로드 큐 (대용량 — 이벤트 JSON 경로와 분리) ──
    struct ClipJob {
        std::string      path;
        BlackboxClipMeta meta;
    };
    mutable std::mutex          clip_mutex_;
    std::queue<ClipJob>         clip_queue_;
    std::condition_variable     clip_cv_;
    std::thread                 clip_thread_;
    std::atomic<bool>           clip_running_{false};

    static std::string ToJson(const DepartureEvent& evt);
    static std::string DumpingToJson(const DumpingEvent& evt, const std::string& image_base64,
                                     int zone, bool restricted_zone,
                                     const std::string& confidence = "",
                                     int validated_frames = -1,
                                     int validate_window = -1);
    static std::string IntrusionToJson(const IntrusionEvent& evt,
                                       const std::string& image_base64);
    static std::string MsToIso8601(uint64_t epoch_ms);

    // ── 큐 엔트리 라우팅 태그 ──────────────────────────────────────
    //   큐/파일에는 "<tag>\t<json>" 으로 저장한다. 'D'=dumping endpoint,
    //   'I'=intrusion endpoint. 태그 없는 레거시 라인은 'D' 로 해석한다.
    static std::string Tagged(char tag, const std::string& json);
    static std::pair<char, std::string> SplitTagged(const std::string& line);
    const std::string& UrlForTag(char tag) const;

    bool HttpPost(const std::string& url, const std::string& json_body);
    bool HttpPostClip(const ClipJob& job);
    void ClipLoop();
    void EnqueueJson(const std::string& json, char tag = 'D');
    void SendLoop();
    // 서버 미설정 상태를 주기적으로 안내 (스팸 방지)
    void MaybeWarnNoServer();

    void AppendToFile(const std::string& json_line);
    std::vector<std::string> ReadQueueFile();
    void WriteQueueFile(const std::vector<std::string>& lines);
    void RetryLoop();
};

} // namespace ecowarden
