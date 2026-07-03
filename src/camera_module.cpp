#include "camera_module.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>

#ifdef USE_OPENCV
#include <chrono>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

static std::string Base64Encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
    // 출력 크기를 미리 계산하여 단일 할당 (기존: char 단위 += 재할당 반복)
    size_t out_len = 4 * ((in_len + 2) / 3);
    std::string ret;
    ret.reserve(out_len);

    int i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];

    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i <4) ; i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while((i++ < 3))
            ret += '=';
    }
    return ret;
}
#endif

namespace ecowarden {

// ── PIMPL: OpenCV 타입은 여기서만 다룬다 ─────────────────────────────
//   USE_OPENCV 가 켜져 있으면 VideoCapture + Mat 을 실제로 보관하고,
//   꺼져 있으면 빈 struct 로 링크되어 컴파일만 되도록 한다 (camera disabled 경로).
struct CameraModule::Impl {
#ifdef USE_OPENCV
    cv::VideoCapture cap;
    cv::Mat          latest_frame;
#endif
};

CameraModule::CameraModule(int device_id)
    : device_id_(device_id),
      capture_dir_("captures"),
      impl_(std::make_unique<Impl>()),
      running_(false),
      frame_ready_(false) {
    mkdir(capture_dir_.c_str(), 0755);
}

CameraModule::~CameraModule() {
    Stop();
    // impl_ 는 unique_ptr 이므로 자동 소멸 → OpenCV 리소스 정리.
}

void CameraModule::SetCaptureDir(const std::string& dir_path) {
    capture_dir_ = dir_path;
    mkdir(capture_dir_.c_str(), 0755);
}

void CameraModule::ConfigureBlackbox(const BlackboxParams& params) {
    blackbox_ = params;
    if (blackbox_.fps <= 0.0) blackbox_.fps = 10.0;
}

bool CameraModule::Start() {
#ifdef USE_OPENCV
    if (running_) return true;

    if (!impl_->cap.open(device_id_, cv::CAP_V4L2)) {
        std::cerr << "[CAMERA] Failed to open device " << device_id_ << "\n";
        return false;
    }

    impl_->cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    impl_->cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    impl_->cap.set(cv::CAP_PROP_BUFFERSIZE, 1); // 지연 방지를 위해 버퍼 크기 최소화

    running_ = true;
    capture_thread_ = std::thread(&CameraModule::CaptureLoop, this);
    return true;
#else
    return false;
#endif
}

void CameraModule::Stop() {
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (blackbox_thread_.joinable()) {
        blackbox_thread_.join();
    }
#ifdef USE_OPENCV
    if (impl_ && impl_->cap.isOpened()) {
        impl_->cap.release();
    }
#endif
}

#ifdef USE_OPENCV
static int64_t NowEpochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
#endif

void CameraModule::CaptureLoop() {
#ifdef USE_OPENCV
    cv::Mat temp_frame;

    while (running_) {
        if (impl_->cap.read(temp_frame)) {
            if (!temp_frame.empty()) {
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    temp_frame.copyTo(impl_->latest_frame);
                    frame_ready_ = true;
                }
                frame_cv_.notify_all();

                // 블랙박스 링버퍼: fps 간격으로만 JPEG 압축해 보관.
                // 저장 worker 가 post 구간까지 읽을 수 있도록 pre+post 초를 유지한다.
                if (blackbox_.enable) {
                    const int64_t now_ms = NowEpochMs();
                    if (now_ms >= next_blackbox_ms_) {
                        next_blackbox_ms_ =
                            now_ms + static_cast<int64_t>(1000.0 / blackbox_.fps);
                        std::vector<uchar> jpeg;
                        std::vector<int> params = {
                            cv::IMWRITE_JPEG_QUALITY, blackbox_.jpeg_quality};
                        if (cv::imencode(".jpg", temp_frame, jpeg, params)) {
                            const int64_t keep_ms =
                                static_cast<int64_t>(blackbox_.pre_seconds +
                                                     blackbox_.post_seconds + 2) * 1000;
                            std::lock_guard<std::mutex> lock(blackbox_mutex_);
                            blackbox_buf_.push_back({now_ms, std::move(jpeg)});
                            while (!blackbox_buf_.empty() &&
                                   blackbox_buf_.front().ts_ms < now_ms - keep_ms) {
                                blackbox_buf_.pop_front();
                            }
                        }
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // ~30fps 이상 필요 없음
    }
#endif
}

bool CameraModule::RequestBlackboxSave(const std::string& path) {
#ifdef USE_OPENCV
    if (!blackbox_.enable || path.empty()) return false;
    if (!running_) return false;

    if (blackbox_saving_.exchange(true)) {
        std::cerr << "[CAMERA] Blackbox save already in progress — skipped: "
                  << path << "\n";
        return false;
    }
    if (blackbox_thread_.joinable()) blackbox_thread_.join(); // 종료된 이전 worker 회수

    blackbox_thread_ = std::thread(&CameraModule::BlackboxSaveWorker, this,
                                   path, NowEpochMs());
    return true;
#else
    (void)path;
    return false;
#endif
}

void CameraModule::BlackboxSaveWorker(std::string path, int64_t event_ms) {
#ifdef USE_OPENCV
    const int64_t begin_ms = event_ms -
        static_cast<int64_t>(blackbox_.pre_seconds) * 1000;
    const int64_t end_ms = event_ms +
        static_cast<int64_t>(blackbox_.post_seconds) * 1000;

    // post 구간 프레임이 버퍼에 쌓일 때까지 대기. 카메라가 멈추면 3초 여유 후 탈출.
    while (running_) {
        {
            std::lock_guard<std::mutex> lock(blackbox_mutex_);
            if (!blackbox_buf_.empty() && blackbox_buf_.back().ts_ms >= end_ms) break;
        }
        if (NowEpochMs() > end_ms + 3000) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<BlackboxFrame> frames;
    {
        std::lock_guard<std::mutex> lock(blackbox_mutex_);
        for (const auto& f : blackbox_buf_) {
            if (f.ts_ms >= begin_ms && f.ts_ms <= end_ms) frames.push_back(f);
        }
    }

    if (frames.empty()) {
        std::cerr << "[CAMERA] Blackbox save failed: no buffered frames\n";
        blackbox_saving_ = false;
        return;
    }

    const std::filesystem::path out_path(path);
    if (out_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);
    }

    cv::Mat first = cv::imdecode(frames.front().jpeg, cv::IMREAD_COLOR);
    if (first.empty()) {
        std::cerr << "[CAMERA] Blackbox save failed: decode error\n";
        blackbox_saving_ = false;
        return;
    }

    cv::VideoWriter writer(path,
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           blackbox_.fps, first.size());
    if (!writer.isOpened()) {
        std::cerr << "[CAMERA] Blackbox save failed: cannot open " << path << "\n";
        blackbox_saving_ = false;
        return;
    }

    size_t written = 0;
    for (const auto& f : frames) {
        cv::Mat img = cv::imdecode(f.jpeg, cv::IMREAD_COLOR);
        if (img.empty()) continue;
        if (img.size() != first.size()) cv::resize(img, img, first.size());
        writer.write(img);
        written++;
    }
    writer.release();

    std::cout << "[CAMERA] Blackbox clip saved: " << path
              << " (" << written << " frames, "
              << (end_ms - begin_ms) / 1000 << "s window)\n";
    blackbox_saving_ = false;
#else
    (void)path;
    (void)event_ms;
#endif
}

std::string CameraModule::CaptureBase64() {
#ifdef USE_OPENCV
    if (!frame_ready_) return "";

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        impl_->latest_frame.copyTo(frame);
    }

    if (frame.empty()) return "";

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", frame, buf, params)) return "";

    return Base64Encode(buf.data(), buf.size());
#else
    return "";
#endif
}

std::string CameraModule::CaptureToFile() {
#ifdef USE_OPENCV
    if (!frame_ready_) {
        std::cerr << "[CAMERA] No frame ready yet.\n";
        return "";
    }

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        impl_->latest_frame.copyTo(frame);
    }

    if (frame.empty()) return "";

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    struct tm tm_buf{};
    localtime_r(&in_time_t, &tm_buf);
    std::stringstream ss;
    ss << capture_dir_ << "/capture_"
       << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
       << "_" << std::setw(3) << std::setfill('0') << ms << ".jpg";
    std::string filename = ss.str();

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    if (cv::imwrite(filename, frame, params)) {
        std::cout << "[CAMERA] Async snapshot saved: " << filename << "\n";
        return filename;
    }
#endif
    return "";
}

std::string CameraModule::CaptureToFilePath(const std::string& path,
                                            const std::string& annotation) {
#ifdef USE_OPENCV
    if (!frame_ready_) {
        std::cerr << "[CAMERA] No frame ready yet.\n";
        return "";
    }
    if (path.empty()) return "";

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        impl_->latest_frame.copyTo(frame);
    }

    if (frame.empty()) return "";

    // 증거 사진 배너: 사진만 단독 유통돼도 트랙 ID/존/시각을 식별할 수 있게 한다.
    if (!annotation.empty()) {
        constexpr int kPad = 8;
        constexpr double kScale = 0.8;
        constexpr int kThickness = 2;
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            annotation, cv::FONT_HERSHEY_SIMPLEX, kScale, kThickness, &baseline);
        cv::rectangle(frame, cv::Point(0, 0),
                      cv::Point(text_size.width + kPad * 2,
                                text_size.height + baseline + kPad * 2),
                      cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, annotation,
                    cv::Point(kPad, kPad + text_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, kScale,
                    cv::Scalar(255, 255, 255), kThickness);
    }

    const std::filesystem::path out_path(path);
    if (out_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);
    }

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    if (cv::imwrite(path, frame, params)) {
        std::cout << "[CAMERA] Snapshot saved: " << path << "\n";
        return path;
    }
#else
    (void)path;
    (void)annotation;
#endif
    return "";
}

bool CameraModule::ShowPreviewFrame(const std::string& window_name, int wait_ms) {
#ifdef USE_OPENCV
    if (!frame_ready_) return true;

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        impl_->latest_frame.copyTo(frame);
    }

    if (frame.empty()) return true;

    try {
        cv::imshow(window_name.empty() ? "EcoWarden Servo Sweep" : window_name,
                   frame);
        const int key = cv::waitKey(std::max(wait_ms, 1));
        return key != 27 && key != 'q' && key != 'Q';
    } catch (const cv::Exception& e) {
        std::cerr << "[CAMERA] Preview unavailable: " << e.what() << "\n";
        return false;
    }
#else
    (void)window_name;
    (void)wait_ms;
    return false;
#endif
}

} // namespace ecowarden
