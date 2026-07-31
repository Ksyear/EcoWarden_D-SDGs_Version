#pragma once

#include "evidence_vault.h"
#include "face_masking.h"
#include "head_label.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ecowarden {

// ── 블랙박스(전후 N초) 링버퍼 설정 ──────────────────────────────────
//   캡처 스레드가 최근 프레임을 JPEG 로 압축해 메모리에 보관하다가,
//   이벤트 발생 시 [이벤트-pre초, 이벤트+post초] 구간을 AVI 로 저장한다.
//   기본 10초/10fps/quality 75 기준 버퍼 메모리는 약 40~60MB (1080p).
struct BlackboxParams {
    bool     enable       = true;
    uint32_t pre_seconds  = 10;
    uint32_t post_seconds = 10;
    double   fps          = 10.0;
    int      jpeg_quality = 75;

    // 동시에 저장할 수 있는 클립 수.
    //
    //   한 번 저장하는 데 post_seconds(기본 10초) + 인코딩 + 얼굴 마스킹
    //   시간이 걸린다. 동시 1개만 허용하면 그 사이에 확정된 투기의 영상이
    //   통째로 버려진다 — 실제로 현장 로그에서 확정된 투기(object 49)의
    //   블랙박스가 "already in progress" 로 skip 됐다.
    //   메모리는 클립당 대략 40~60MB(1080p JPEG 200장) 이므로 2~3 이 상한.
    uint32_t max_concurrent_saves = 2;
};

// ── 블랙박스 클립 메타 — 서버 업로드 시 이벤트 레코드와 연결하는 키 ──
struct BlackboxClipMeta {
    std::string event_type;        // "intrusion" | "dumping"
    uint32_t    person_id = 0;
    uint32_t    object_id = 0;     // dumping 전용 (0 = 없음)
    int         zone = -1;
    int64_t     event_time_ns = 0; // 이벤트 JSON 의 event_time 과 동일 값
};

// 클립 저장 완료 콜백. 저장 worker 스레드에서 호출되므로 콜백 구현은
// 스레드 안전해야 한다 (EventNotifier::UploadClip 은 큐 push 만 하므로 안전).
using BlackboxSavedCallback =
    std::function<void(const std::string& path, const BlackboxClipMeta& meta)>;

/**
 * @brief 비동기 프레임 캡처를 지원하는 카메라 모듈
 *
 *   OpenCV 헤더는 public header 에 노출하지 않는다(PIMPL):
 *   - 상위 모듈은 opencv2/opencv.hpp 를 포함할 필요 없음
 *   - 원본 구현의 raw void* 캐스팅(+수동 new/delete) 제거
 *   - USE_OPENCV 가 꺼져 있으면 Impl 이 빈 struct → 아무 것도 하지 않는다.
 */
class CameraModule {
public:
    explicit CameraModule(int device_id = 0);
    ~CameraModule();

    CameraModule(const CameraModule&) = delete;
    CameraModule& operator=(const CameraModule&) = delete;

    // 캡처 이미지를 저장할 디렉토리 설정 (기본: "captures")
    void SetCaptureDir(const std::string& dir_path);

    // 블랙박스 링버퍼 설정 — Start() 전에 호출한다.
    void ConfigureBlackbox(const BlackboxParams& params);

    // 머리 위 ID 라벨 설정 (env 연결 값은 production_params.h 의
    // DefaultHeadLabelParams() 사용. 미호출 시 기본값으로 동작).
    void ConfigureHeadLabel(const HeadLabelParams& params);

    /**
     * @brief 얼굴 마스킹 설정 — Start() 전에 호출한다.
     *
     *   프라이버시 2계층. 저장·전송되는 모든 프레임에 적용된다.
     *   fail-closed 이므로, 마스킹이 요구됐는데 검출기가 없고 폴백도
     *   꺼져 있으면 **사진을 저장하지 않는다**(빈 문자열 반환).
     */
    void ConfigureFaceMask(const FaceMaskParams& params);

    // 마스킹 백엔드 이름 ("dnn" | "haar" | "fallback" | "none")
    const char* FaceMaskBackend() const;

    /**
     * @brief 원본 암호화 보관소 연결 — Start() 전에 호출한다.
     *
     *   프라이버시 3계층. 설정하면 **마스킹 전** 원본을 vault 에
     *   암호화 저장한다. 소유권은 넘기지 않으므로 vault 의 수명이
     *   CameraModule 보다 길어야 한다. nullptr 이면 원본을 남기지 않는다.
     */
    void SetEvidenceVault(EvidenceVault* vault);

    // 스레드 시작/정지
    bool Start();
    void Stop();

    // 즉시 촬영 (메모리에 저장된 최신 프레임 사용)
    std::string CaptureBase64();
    std::string CaptureToFile();
    // annotation 이 비어 있지 않으면 사진 좌상단에 ID/존/시각 배너를 그린다.
    // head_text 가 비어 있지 않으면 사람 머리 위 추정 위치에 ID 라벨을
    // 추가로 그린다. head_offset_deg 는 서보 조준각 대비 사람의 수평
    // 각도 차, head_distance_mm 는 사람까지 거리 (head_label.h 참조).
    std::string CaptureToFilePath(const std::string& path,
                                  const std::string& annotation = "",
                                  const std::string& head_text = "",
                                  double head_offset_deg = 0.0,
                                  double head_distance_mm = 0.0);
    bool ShowPreviewFrame(const std::string& window_name, int wait_ms = 1);

    /**
     * @brief 이벤트 시점 기준 전 pre초 + 후 post초 구간을 AVI 로 비동기 저장.
     * @param meta 저장 완료 콜백으로 전달되는 이벤트 연계 정보 (서버 업로드용)
     * @return 저장 worker 가 시작되면 true. 이미 저장 중이거나 블랙박스가
     *         비활성이면 false (scan loop 는 어느 쪽이든 블로킹되지 않음).
     */
    bool RequestBlackboxSave(const std::string& path,
                             const BlackboxClipMeta& meta = {});

    // 클립 저장 완료 시 호출할 콜백 등록 — Start() 전에 호출한다.
    void SetBlackboxSavedCallback(BlackboxSavedCallback cb);

private:
    void CaptureLoop();
    void BlackboxSaveWorker(std::string path, int64_t event_ms,
                            BlackboxClipMeta meta);

    /**
     * @brief 증거 프레임 공통 전처리 — ① 원본 vault 보관 ② 얼굴 마스킹.
     * @param frame_mat  `cv::Mat*`
     * @param vault_name vault 파일명 베이스 (비면 원본 보관 생략)
     * @return 이 프레임을 저장·전송해도 되는가 (fail-closed 판정 결과)
     *
     *   반드시 배너·ID 라벨을 그리기 **전에** 호출해야 라벨이 블러되지 않는다.
     */
    bool PrepareEvidenceFrame(void* frame_mat, const std::string& vault_name,
                              FaceMaskResult* out_mask = nullptr);

    // 마스킹 번호·좌표를 `<image_path>.masks.json` 으로 남긴다.
    void WriteMaskSidecar(const std::string& image_path,
                          const FaceMaskResult& mask);

    int device_id_;
    std::string capture_dir_;

    // OpenCV 상태(cv::VideoCapture, cv::Mat)는 Impl 내부에 숨긴다.
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::thread capture_thread_;
    std::mutex frame_mutex_;
    std::atomic<bool> running_;
    std::atomic<bool> frame_ready_;
    std::condition_variable frame_cv_;

    // ── 블랙박스 링버퍼 (JPEG 압축 프레임, OpenCV 타입 비의존) ──────
    struct BlackboxFrame {
        int64_t ts_ms;
        std::vector<unsigned char> jpeg;
    };
    BlackboxParams blackbox_;
    HeadLabelParams head_label_;

    // ── 프라이버시 2·3계층 ──────────────────────────────────────────
    FaceMaskParams face_mask_params_;
    std::unique_ptr<FaceMasker> face_masker_;   // 사진 경로 (메인 루프 스레드)
    // 블랙박스 저장 worker 전용 — 검출기가 스레드 안전하지 않아 분리한다.
    std::unique_ptr<FaceMasker> blackbox_masker_;
    EvidenceVault* evidence_vault_ = nullptr;   // 소유하지 않음

    BlackboxSavedCallback blackbox_saved_cb_;
    std::mutex blackbox_mutex_;
    std::deque<BlackboxFrame> blackbox_buf_;
    int64_t next_blackbox_ms_ = 0;
    // 동시 저장 지원 (v56): 단일 스레드였을 때 확정된 투기의 영상이
    // 다른 저장 때문에 버려지는 문제가 있었다.
    std::mutex blackbox_thread_mutex_;
    std::vector<std::thread> blackbox_threads_;
    std::atomic<uint32_t> blackbox_active_{0};
};

} // namespace ecowarden
