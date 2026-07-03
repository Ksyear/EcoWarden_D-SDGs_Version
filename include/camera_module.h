#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
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
};

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

    // 스레드 시작/정지
    bool Start();
    void Stop();

    // 즉시 촬영 (메모리에 저장된 최신 프레임 사용)
    std::string CaptureBase64();
    std::string CaptureToFile();
    // annotation 이 비어 있지 않으면 사진 좌상단에 ID/존/시각 배너를 그린다.
    std::string CaptureToFilePath(const std::string& path,
                                  const std::string& annotation = "");
    bool ShowPreviewFrame(const std::string& window_name, int wait_ms = 1);

    /**
     * @brief 이벤트 시점 기준 전 pre초 + 후 post초 구간을 AVI 로 비동기 저장.
     * @return 저장 worker 가 시작되면 true. 이미 저장 중이거나 블랙박스가
     *         비활성이면 false (scan loop 는 어느 쪽이든 블로킹되지 않음).
     */
    bool RequestBlackboxSave(const std::string& path);

private:
    void CaptureLoop();
    void BlackboxSaveWorker(std::string path, int64_t event_ms);

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
    std::mutex blackbox_mutex_;
    std::deque<BlackboxFrame> blackbox_buf_;
    int64_t next_blackbox_ms_ = 0;
    std::thread blackbox_thread_;
    std::atomic<bool> blackbox_saving_{false};
};

} // namespace ecowarden
