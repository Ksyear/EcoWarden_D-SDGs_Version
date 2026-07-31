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
 * @file  event_notifier.cpp
 * @brief 비동기 HTTP POST 이벤트 전송 + JSONL 파일 큐 + 백그라운드 재전송
 * @date  2026
 */

#include <nlohmann/json.hpp>
#include "event_notifier.h"
#include "time_utils.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <curl/curl.h>

namespace ecowarden {

// ── curl 응답 콜백 (본문 무시) ──────────────────────────────────────
static size_t CurlWriteDiscard(void* /*ptr*/, size_t size, size_t nmemb,
                               void* /*userdata*/) {
    return size * nmemb;
}

// ── curl_global_init 중복 방지 (프로세스 전체에서 1회만) ─────────────
static std::once_flag s_curl_init_flag;

// ── 생성자 / 소멸자 ─────────────────────────────────────────────────
EventNotifier::EventNotifier(const NotifierConfig& cfg) : cfg_(cfg) {
    std::call_once(s_curl_init_flag, []() {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    });

    // API 키 주입: config → 환경변수. 레거시 하드코딩 키는 v56 에서 제거했다.
    //
    //   이전 버전은 키가 없으면 저장소 이력에 노출된 키로 조용히 폴백했다.
    //   그 키는 공개 저장소에서 누구나 읽을 수 있으므로 "인증"이 아니었고,
    //   경고 로그를 띄워도 운영 중엔 아무도 보지 않는다. 이제는 키가 없으면
    //   **전송을 시도하지 않고 로컬 큐에만 쌓는다** — 사고가 조용히
    //   지나가는 대신 눈에 띄게 만든다 (fail-closed).
    if (cfg_.api_key.empty()) {
        if (const char* env = std::getenv("ECOWARDEN_API_KEY")) {
            if (env[0] != '\0') cfg_.api_key = env;
        }
    }
    if (cfg_.api_key.empty()) {
        std::fprintf(stderr,
            "[NOTIFIER] ⚠ ECOWARDEN_API_KEY 미설정 — 서버 전송을 비활성화합니다.\n"
            "[NOTIFIER]   이벤트는 로컬 큐(%s)에만 쌓입니다.\n"
            "[NOTIFIER]   설정: /etc/ecowarden/secrets.conf 에\n"
            "[NOTIFIER]         ECOWARDEN_API_KEY=<서버에서 발급한 키>\n",
            cfg_.queue_file_path.c_str());
    }

    // 큐 파일 경로 환경변수 오버라이드 (/tmp 는 모든 사용자가 읽을 수 있음 —
    // 운영 배포에서는 ECOWARDEN_QUEUE_FILE=/var/lib/ecowarden/event_queue.jsonl 권장)
    if (const char* env = std::getenv("ECOWARDEN_QUEUE_FILE")) {
        if (env[0] != '\0') cfg_.queue_file_path = env;
    }

    // 금지구역 침입 이벤트 endpoint 결정: env → config → dumping URL 치환 순.
    if (const char* env = std::getenv("ECOWARDEN_INTRUSION_URL")) {
        if (env[0] != '\0') cfg_.intrusion_url = env;
    }
    if (cfg_.intrusion_url.empty()) {
        cfg_.intrusion_url = cfg_.endpoint_url;
        const std::string from = "dumping-event";
        const size_t pos = cfg_.intrusion_url.find(from);
        if (pos != std::string::npos) {
            cfg_.intrusion_url.replace(pos, from.size(), "intrusion-event");
        }
    }
    if (const char* env = std::getenv("ECOWARDEN_INTRUSION_NOTIFY")) {
        if (env[0] != '\0') {
            cfg_.intrusion_notify =
                (env[0] == '1' || env[0] == 't' || env[0] == 'T' ||
                 env[0] == 'y' || env[0] == 'Y');
        }
    }

    // 증거 영상 클립 업로드 endpoint 결정: env → config → dumping URL 치환 순.
    if (const char* env = std::getenv("ECOWARDEN_CLIP_URL")) {
        if (env[0] != '\0') cfg_.clip_url = env;
    }
    if (cfg_.clip_url.empty()) {
        cfg_.clip_url = cfg_.endpoint_url;
        const std::string from = "dumping-event";
        const size_t pos = cfg_.clip_url.find(from);
        if (pos != std::string::npos) {
            cfg_.clip_url.replace(pos, from.size(), "evidence-clip");
        }
    }
    if (const char* env = std::getenv("ECOWARDEN_CLIP_UPLOAD")) {
        if (env[0] != '\0') {
            cfg_.clip_upload =
                (env[0] == '1' || env[0] == 't' || env[0] == 'T' ||
                 env[0] == 'y' || env[0] == 'Y');
        }
    }
    if (const char* env = std::getenv("ECOWARDEN_CLIP_TIMEOUT_MS")) {
        const long v = std::strtol(env, nullptr, 10);
        if (v > 0) cfg_.clip_timeout_ms = v;
    }

    // dry_run 상태를 시작 시 큰 소리로 알린다. (이전에는 HttpPost가
    // 아무 말 없이 무동작 상태였어서 운영자가 알 수 없었음)
    if (cfg_.dry_run) {
        std::fprintf(stderr,
            "[NOTIFIER] ⚠ DRY-RUN MODE — 모든 이벤트가 drop되며 HTTP POST가 호출되지 않습니다.\n"
            "[NOTIFIER] dry_run=false 로 설정해야 실제 전송이 일어납니다.\n");
    } else {
        std::fprintf(stderr,
            "[NOTIFIER] HTTP POST 활성 — endpoint=%s timeout=%ldms\n",
            cfg_.endpoint_url.c_str(), cfg_.timeout_ms);
    }
}

EventNotifier::~EventNotifier() {
    StopRetryThread();
    // curl_global_cleanup은 프로세스 종료 시 자동 정리에 맡김
    // (여러 EventNotifier 인스턴스 생성 가능성 대비)
}

// NowMs / MsToIso8601 / MsToApiTime 은 time_utils.h 의 free function 사용.
// ── EventNotifier::MsToIso8601 (legacy static member) ─── 호환용 wrapper
std::string EventNotifier::MsToIso8601(uint64_t epoch_ms) {
    return ecowarden::MsToIso8601(epoch_ms);
}

// (이전 PostGIS WKB hex 인코더는 현재 백엔드 스펙에서 사용되지 않아 제거됨 —
//  필요해지면 git history 에서 복구 가능)

// ── JSON 문자열 이스케이프 ──────────────────────────────────────────
static std::string EscapeJson(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;      break;
        }
    }
    return out;
}

// ── DepartureEvent → JSON ───────────────────────────────────────────
std::string EventNotifier::ToJson(const DepartureEvent& evt) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{"
            "\"timestamp\":\"%s\","
            "\"x\":%.3f,"
            "\"y\":%.3f,"
            "\"cluster_id\":%u,"
            "\"type\":\"abandoned\""
        "}",
        EscapeJson(MsToIso8601(evt.timestamp_ms)).c_str(),
        evt.x_mm / 1000.0,
        evt.y_mm / 1000.0,
        evt.track_id
    );
    return buf;
}

// ── DumpingEvent → JSON (API SPEC: eco-warden backend) ──────────────
std::string EventNotifier::DumpingToJson(const DumpingEvent& evt, const std::string& image_base64,
                                         int zone, bool restricted_zone,
                                         const std::string& confidence,
                                         int validated_frames, int validate_window) {
    nlohmann::json j;

    j["person_id"]       = static_cast<int>(evt.person_track_id);
    j["person_x"]        = static_cast<float>(evt.person_x_mm / 1000.0);
    j["person_y"]        = static_cast<float>(evt.person_y_mm / 1000.0);
    j["cumulative_dist"] = static_cast<float>(evt.person_cumulative_dist_mm / 1000.0);

    j["object_id"]       = static_cast<int>(evt.object_track_id);
    j["object_x"]        = static_cast<float>(evt.object_x_mm / 1000.0);
    j["object_y"]        = static_cast<float>(evt.object_y_mm / 1000.0);

    // event_time: Unix Timestamp (nanoseconds)
    // evt.timestamp_ms is milliseconds, convert to nanoseconds
    j["event_time"]      = static_cast<int64_t>(evt.timestamp_ms) * 1000000LL;

    // 존 정보(0~4)와 금지구역 여부. zone<0 이면 생략 (레거시 페이로드와 동일).
    if (zone >= 0) {
        j["zone"]     = zone;
        j["severity"] = restricted_zone ? "high" : "normal";
    }

    // 확정 후 재검증 결과 (dump_validation.h) — additive 필드라 서버가
    // 무시해도 무방. confidence 비어 있으면 레거시 페이로드와 동일.
    if (!confidence.empty()) {
        j["confidence"] = confidence;
        if (validated_frames >= 0 && validate_window > 0) {
            j["validated_frames"] = validated_frames;
            j["validate_window"]  = validate_window;
        }
    }

    // dumping 과 동일하게, 이미지가 없으면 null 을 명시한다.
    if (!image_base64.empty()) {
        j["image_base64"] = image_base64;
    } else {
        j["image_base64"] = nullptr;
    }

    return j.dump();
}

// ── IntrusionEvent → JSON ───────────────────────────────────────────
std::string EventNotifier::IntrusionToJson(const IntrusionEvent& evt,
                                           const std::string& image_base64) {
    nlohmann::json j;

    // v56: 분류기가 붙으면 event_type 이 unknown_object /
    //      animal_or_small_object 로 갈릴 수 있다. 레거시 서버가 "type"
    //      만 보고 분기하던 동작을 깨지 않도록 기본값은 "intrusion".
    j["type"]       = (evt.event_type && evt.event_type[0]) ? evt.event_type
                                                            : "intrusion";
    j["person_id"]  = static_cast<int>(evt.person_track_id);
    j["person_x"]   = static_cast<float>(evt.person_x_mm / 1000.0);
    j["person_y"]   = static_cast<float>(evt.person_y_mm / 1000.0);
    j["zone"]       = evt.zone;
    j["event_time"] = static_cast<int64_t>(evt.timestamp_ms) * 1000000LL;

    // 분류 결과 — 서버가 무시해도 무방한 추가 필드 (v56)
    if (evt.severity && evt.severity[0]) {
        j["severity"] = evt.severity;
    }
    if (evt.object_class && evt.object_class[0]) {
        j["object_class"] = evt.object_class;
    }
    if (evt.classify_confidence > 0.0) {
        j["classify_confidence"] =
            static_cast<float>(evt.classify_confidence);
    }

    // 백엔드 합의 스키마: 이미지가 없으면 **필드를 생략하지 않고 null** 을 보낸다.
    // (필드 자체가 없으면 스키마 검증을 엄격하게 하는 서버에서 걸린다)
    if (!image_base64.empty()) {
        j["image_base64"] = image_base64;
    } else {
        j["image_base64"] = nullptr;
    }

    return j.dump();
}

// ── libcurl HTTP POST ───────────────────────────────────────────────
//
//   이전엔 cfg_ 에 관계없이 "사용자 요청으로" 스텁 true 반환이었음 →
//   dropped_count_ / sent_count_ 구분 불가 + 운영자가 상태 파악 불가.
//   v4: 정상 curl 경로 복원 + cfg_.dry_run flag 뒤로 격리.
//       dry_run 인 경우 false 를 반환해 SendLoop 쪽 분기에서 pending_queue
//       에 쌓이지 않고 dropped_count_ 증가 후 폐기되도록 한다 (SendLoop 패치 참조).
//
bool EventNotifier::HttpPost(const std::string& url, const std::string& json_body) {
    // v56 fail-closed: 키가 없으면 아예 보내지 않는다. false 를 돌려주면
    // SendLoop 가 로컬 큐에 보존하므로 이벤트가 유실되지는 않는다.
    if (cfg_.api_key.empty()) return false;

    if (cfg_.dry_run) {
        // dry_run 모드: 네트워크 호출하지 않고 명시적 false. SendLoop 가
        // dropped_count_ 를 증가시키고 파일/큐에 쌓지 않도록 처리한다.
        return false;
    }

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    const std::string api_key_header = "X-API-Key: " + cfg_.api_key;
    headers = curl_slist_append(headers, api_key_header.c_str());

    long timeout = (cfg_.timeout_ms > 0) ? cfg_.timeout_ms : 1000;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(json_body.size()));
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, timeout);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteDiscard);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode res = curl_easy_perform(curl);

    bool success = false;
    if (res == CURLE_OK) {
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
        success = (http_code >= 200 && http_code < 300);
        if (!success && fail_count_ % 10 == 0) {
            std::fprintf(stderr, "[NOTIFIER] HTTP %ld (Server Down?)\n", http_code);
        }
    } else {
        if (fail_count_ % 50 == 0) {
            std::fprintf(stderr, "[NOTIFIER] Connection failed (URL: %s)\n",
                         url.c_str());
        }
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return success;
}

// ── 큐 엔트리 태그 인코딩/디코딩 ────────────────────────────────────
std::string EventNotifier::Tagged(char tag, const std::string& json) {
    return std::string(1, tag) + "\t" + json;
}

std::pair<char, std::string> EventNotifier::SplitTagged(const std::string& line) {
    if (line.size() > 2 && line[1] == '\t' &&
        (line[0] == 'D' || line[0] == 'I')) {
        return {line[0], line.substr(2)};
    }
    return {'D', line}; // 태그 없는 레거시 큐 라인은 dumping endpoint 로 처리
}

const std::string& EventNotifier::UrlForTag(char tag) const {
    return (tag == 'I') ? cfg_.intrusion_url : cfg_.endpoint_url;
}

// ── EnqueueJson: 비동기 전송 큐에 추가 (논블로킹) ──────────────────
void EventNotifier::EnqueueJson(const std::string& json, char tag) {
    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        send_queue_.push(Tagged(tag, json));
    }
    send_queue_cv_.notify_one();
}

// ── Send: 비동기 큐에 넣기 (논블로킹) ──────────────────────────────
void EventNotifier::Send(const DepartureEvent& evt) {
    EnqueueJson(ToJson(evt));
}

void EventNotifier::SendDumping(const DumpingEvent& evt, const std::string& image_base64,
                                int zone, bool restricted_zone,
                                const std::string& confidence,
                                int validated_frames, int validate_window) {
    EnqueueJson(DumpingToJson(evt, image_base64, zone, restricted_zone,
                              confidence, validated_frames, validate_window));
}

void EventNotifier::SendIntrusion(const IntrusionEvent& evt,
                                  const std::string& image_base64) {
    if (!cfg_.intrusion_notify) {
        std::fprintf(stderr,
            "[NOTIFIER] intrusion 전송 비활성(ECOWARDEN_INTRUSION_NOTIFY=0) — "
            "person=%u zone=%d 기록만 유지\n",
            evt.person_track_id, evt.zone);
        return;
    }
    EnqueueJson(IntrusionToJson(evt, image_base64), 'I');
}

// ── 증거 영상 클립 업로드 (multipart, 전용 스레드) ──────────────────
//
//   블랙박스 클립은 수십 MB 라 3초 타임아웃의 이벤트 JSON 경로와 분리한다.
//   실패해도 클립 파일은 기기에 그대로 남으므로 재시도 소진 시 drop 한다
//   (이벤트 레코드는 이미 사진과 함께 서버에 있음 — 클립은 보강 증거).
void EventNotifier::UploadClip(const std::string& path,
                               const BlackboxClipMeta& meta) {
    if (path.empty()) return;
    if (!cfg_.clip_upload) {
        std::fprintf(stderr,
            "[NOTIFIER] clip 업로드 비활성(ECOWARDEN_CLIP_UPLOAD=0) — "
            "기기 보관만: %s\n", path.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(clip_mutex_);
        clip_queue_.push(ClipJob{path, meta});
    }

    // 첫 업로드 요청 시 lazy 시작 (클립을 안 쓰는 구성에서 유휴 스레드 방지)
    if (!clip_running_.exchange(true)) {
        clip_thread_ = std::thread(&EventNotifier::ClipLoop, this);
    }
    clip_cv_.notify_one();
}

void EventNotifier::ClipLoop() {
    while (clip_running_) {
        ClipJob job;
        {
            std::unique_lock<std::mutex> lock(clip_mutex_);
            clip_cv_.wait(lock, [this] {
                return !clip_queue_.empty() || !clip_running_;
            });
            if (!clip_running_ && clip_queue_.empty()) break;
            if (clip_queue_.empty()) continue;
            job = std::move(clip_queue_.front());
            clip_queue_.pop();
        }

        bool ok = false;
        for (uint32_t attempt = 1;
             attempt <= std::max(cfg_.clip_max_retries, 1u) && clip_running_;
             ++attempt) {
            ok = HttpPostClip(job);
            if (ok) break;
            std::fprintf(stderr,
                "[NOTIFIER] clip 업로드 실패 (%u/%u): %s\n",
                attempt, std::max(cfg_.clip_max_retries, 1u),
                job.path.c_str());
            // 재시도 대기 — 종료 요청이 오면 즉시 탈출
            std::unique_lock<std::mutex> lock(clip_mutex_);
            clip_cv_.wait_for(lock,
                std::chrono::seconds(cfg_.retry_interval_sec),
                [this] { return !clip_running_.load(); });
        }

        if (ok) {
            std::fprintf(stderr, "[NOTIFIER] clip 업로드 완료: %s\n",
                         job.path.c_str());
        } else {
            std::fprintf(stderr,
                "[NOTIFIER] clip 업로드 포기 — 기기 파일 유지: %s\n",
                job.path.c_str());
        }
    }
}

bool EventNotifier::HttpPostClip(const ClipJob& job) {
    if (cfg_.api_key.empty()) return false;   // v56 fail-closed (HttpPost 와 동일)
    if (cfg_.dry_run) return false;

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    curl_mime* mime = curl_mime_init(curl);

    curl_mimepart* file_part = curl_mime_addpart(mime);
    curl_mime_name(file_part, "file");
    curl_mime_filedata(file_part, job.path.c_str());
    curl_mime_type(file_part, "video/x-msvideo");

    const auto add_field = [mime](const char* name, const std::string& value) {
        curl_mimepart* p = curl_mime_addpart(mime);
        curl_mime_name(p, name);
        curl_mime_data(p, value.c_str(), CURL_ZERO_TERMINATED);
    };
    add_field("event_type", job.meta.event_type);
    add_field("person_id", std::to_string(job.meta.person_id));
    add_field("object_id", std::to_string(job.meta.object_id));
    add_field("zone", std::to_string(job.meta.zone));
    add_field("event_time", std::to_string(job.meta.event_time_ns));

    struct curl_slist* headers = nullptr;
    const std::string api_key_header = "X-API-Key: " + cfg_.api_key;
    headers = curl_slist_append(headers, api_key_header.c_str());
    // Content-Type 은 libcurl 이 multipart boundary 포함해 자동 설정한다.

    curl_easy_setopt(curl, CURLOPT_URL, cfg_.clip_url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, cfg_.clip_timeout_ms);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, cfg_.timeout_ms);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteDiscard);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    const CURLcode res = curl_easy_perform(curl);

    bool success = false;
    if (res == CURLE_OK) {
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
        success = (http_code >= 200 && http_code < 300);
        if (!success) {
            std::fprintf(stderr, "[NOTIFIER] clip HTTP %ld (URL: %s)\n",
                         http_code, cfg_.clip_url.c_str());
        }
    } else if (res == CURLE_READ_ERROR) {
        std::fprintf(stderr, "[NOTIFIER] clip 파일 읽기 실패: %s\n",
                     job.path.c_str());
    }

    curl_slist_free_all(headers);
    curl_mime_free(mime);
    curl_easy_cleanup(curl);
    return success;
}

// ── SendLoop: 전송 전용 스레드 루프 ─────────────────────────────────
void EventNotifier::SendLoop() {
    while (send_running_) {
        std::string line;

        {
            std::unique_lock<std::mutex> lock(send_queue_mutex_);
            send_queue_cv_.wait(lock, [this] {
                return !send_queue_.empty() || !send_running_;
            });

            if (!send_running_ && send_queue_.empty()) break;
            if (send_queue_.empty()) continue;

            line = std::move(send_queue_.front());
            send_queue_.pop();
        }

        const auto [tag, json] = SplitTagged(line);
        if (HttpPost(UrlForTag(tag), json)) {
            sent_count_++;
        } else if (cfg_.dry_run) {
            // dry_run: 전송 시도 자체를 안 한 것이므로 실패/큐잉이 아닌 drop 으로 회계.
            dropped_count_++;
        } else {
            fail_count_++;
            {
                std::lock_guard<std::mutex> lock(fail_queue_mutex_);
                pending_queue_.push(line);
            }
            AppendToFile(line);
        }
    }

    // 종료 시 큐에 남은 것 모두 처리
    std::lock_guard<std::mutex> lock(send_queue_mutex_);
    while (!send_queue_.empty()) {
        const std::string& line = send_queue_.front();
        const auto [tag, json] = SplitTagged(line);
        if (HttpPost(UrlForTag(tag), json)) {
            sent_count_++;
        } else {
            fail_count_++;
            std::lock_guard<std::mutex> flock(fail_queue_mutex_);
            pending_queue_.push(line);
            AppendToFile(line);
        }
        send_queue_.pop();
    }
}

// ── 로컬 파일 큐: 추가 ─────────────────────────────────────────────
void EventNotifier::AppendToFile(const std::string& json_line) {
    std::ofstream ofs(cfg_.queue_file_path, std::ios::app);
    if (ofs.is_open()) {
        ofs << json_line << "\n";
    } else {
        std::fprintf(stderr, "[NOTIFIER] Cannot open queue file: %s\n",
                     cfg_.queue_file_path.c_str());
    }
}

// ── 로컬 파일 큐: 읽기 ─────────────────────────────────────────────
std::vector<std::string> EventNotifier::ReadQueueFile() {
    std::vector<std::string> lines;
    std::ifstream ifs(cfg_.queue_file_path);
    if (!ifs.is_open()) return lines;

    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

// ── 로컬 파일 큐: 덮어쓰기 (남은 라인만 보존) ─────────────────────
void EventNotifier::WriteQueueFile(const std::vector<std::string>& lines) {
    std::ofstream ofs(cfg_.queue_file_path, std::ios::trunc);
    if (!ofs.is_open()) return;

    for (const auto& l : lines) {
        ofs << l << "\n";
    }
}

// ── FlushQueue: 파일 + 메모리 큐 일괄 재전송 ───────────────────────
size_t EventNotifier::FlushQueue() {
    std::vector<std::string> file_lines = ReadQueueFile();

    {
        std::lock_guard<std::mutex> lock(fail_queue_mutex_);
        while (!pending_queue_.empty()) {
            file_lines.push_back(pending_queue_.front());
            pending_queue_.pop();
        }
    }

    if (file_lines.empty()) return 0;

    std::sort(file_lines.begin(), file_lines.end());
    file_lines.erase(std::unique(file_lines.begin(), file_lines.end()),
                     file_lines.end());

    std::vector<std::string> still_failed;
    size_t success_count = 0;

    for (const auto& line : file_lines) {
        const auto [tag, json] = SplitTagged(line);
        bool ok = false;
        for (uint32_t attempt = 0; attempt < cfg_.max_retries; ++attempt) {
            if (HttpPost(UrlForTag(tag), json)) {
                ok = true;
                break;
            }
        }

        if (ok) {
            success_count++;
            sent_count_++;
        } else {
            still_failed.push_back(line);
        }
    }

    WriteQueueFile(still_failed);

    if (!still_failed.empty()) {
        std::fprintf(stderr, "[NOTIFIER] %zu events still queued after flush\n",
                     still_failed.size());
    }

    return success_count;
}

// ── GetQueueSize ────────────────────────────────────────────────────
size_t EventNotifier::GetQueueSize() const {
    std::lock_guard<std::mutex> lock(fail_queue_mutex_);
    return pending_queue_.size();
}

// ── 스레드 시작/중지 ────────────────────────────────────────────────
void EventNotifier::StartRetryThread() {
    // 전송 전용 스레드 시작
    if (!send_running_) {
        send_running_ = true;
        send_thread_ = std::thread(&EventNotifier::SendLoop, this);
    }

    // 재전송 스레드 시작
    if (!retry_running_) {
        retry_running_ = true;
        retry_thread_ = std::thread(&EventNotifier::RetryLoop, this);
    }
}

void EventNotifier::StopRetryThread() {
    // 전송 스레드 중지
    if (send_running_) {
        send_running_ = false;
        send_queue_cv_.notify_all();
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
    }

    // 재전송 스레드 중지
    if (retry_running_) {
        retry_running_ = false;
        retry_cv_.notify_all();
        if (retry_thread_.joinable()) {
            retry_thread_.join();
        }
    }

    // 클립 업로드 스레드 중지 (UploadClip 첫 호출 시 lazy 시작됨)
    if (clip_running_) {
        clip_running_ = false;
        clip_cv_.notify_all();
        if (clip_thread_.joinable()) {
            clip_thread_.join();
        }
    }
}

void EventNotifier::RetryLoop() {
    while (retry_running_) {
        {
            std::unique_lock<std::mutex> lock(retry_cv_mutex_);
            retry_cv_.wait_for(lock,
                std::chrono::seconds(cfg_.retry_interval_sec),
                [this]{ return !retry_running_.load(); }
            );
        }

        if (!retry_running_) break;

        size_t flushed = FlushQueue();
        if (flushed > 0) {
            std::printf("[NOTIFIER] Background flush: %zu events resent\n",
                        flushed);
        }
    }
}

} // namespace ecowarden
