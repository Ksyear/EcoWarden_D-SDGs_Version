#include "camera_module.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <fstream>

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

void CameraModule::ConfigureHeadLabel(const HeadLabelParams& params) {
    head_label_ = params;
}

void CameraModule::SetBlackboxSavedCallback(BlackboxSavedCallback cb) {
    blackbox_saved_cb_ = std::move(cb);
}

void CameraModule::ConfigureFaceMask(const FaceMaskParams& params) {
    face_mask_params_ = params;
    face_masker_.reset();   // 다음 Start()/사용 시 새 파라미터로 재생성
}

const char* CameraModule::FaceMaskBackend() const {
    return face_masker_ ? face_masker_->BackendName() : "none";
}

void CameraModule::SetEvidenceVault(EvidenceVault* vault) {
    evidence_vault_ = vault;
}

// ── 증거 프레임 공통 전처리 (원본 보관 → 마스킹) ────────────────────
//
//   순서가 중요하다:
//     1) 원본을 먼저 vault 에 암호화 저장한다 (마스킹 전 상태여야 3계층의
//        의미가 있다).
//     2) 그 다음 얼굴을 마스킹한다.
//     3) 호출부가 배너·ID 라벨을 그린다 — 마스킹 후여야 라벨이 안 뭉갠다.
//
bool CameraModule::PrepareEvidenceFrame(void* frame_mat,
                                        const std::string& vault_name,
                                        FaceMaskResult* out_mask) {
#ifdef USE_OPENCV
    if (frame_mat == nullptr) return false;
    cv::Mat& frame = *static_cast<cv::Mat*>(frame_mat);
    if (frame.empty()) return false;

    // ① 원본 암호화 보관 (프라이버시 3계층)
    if (evidence_vault_ != nullptr && evidence_vault_->Available() &&
        !vault_name.empty()) {
        std::vector<uchar> raw;
        const std::vector<int> q = {cv::IMWRITE_JPEG_QUALITY, 95};
        if (cv::imencode(".jpg", frame, raw, q)) {
            const VaultStoreResult vr = evidence_vault_->StoreOriginal(
                vault_name, raw.data(), raw.size());
            if (!vr.ok) {
                std::cerr << "[VAULT] 원본 보관 실패: " << vr.error << "\n";
            }
        }
    }

    // ② 얼굴 마스킹 (프라이버시 2계층)
    if (!face_masker_) {
        face_masker_ = std::make_unique<FaceMasker>(face_mask_params_);
    }
    const FaceMaskResult mr = face_masker_->Apply(&frame);
    if (out_mask) *out_mask = mr;
    if (!mr.safe_to_emit) {
        std::cerr << "[FACEMASK] fail-closed — 마스킹 불가로 증거 사진 저장을 "
                     "건너뜁니다 (ECOWARDEN_FACE_MASK_FALLBACK=1 로 폴백 허용 "
                     "또는 ECOWARDEN_FACE_MASK=0 으로 비활성)\n";
        return false;
    }
    if (mr.applied) {
        std::cout << "[FACEMASK] " << mr.backend << " — 얼굴 " << mr.faces
                  << "개 마스킹"
                  << (mr.used_fallback ? " (상단 영역 폴백)" : "");
        for (const auto& b : mr.boxes) {
            std::cout << " [" << face_mask_params_.label_prefix << b.index
                      << " " << b.x << "," << b.y << " " << b.w << "x" << b.h
                      << "]";
        }
        std::cout << "\n";
    }
    return true;
#else
    (void)frame_mat;
    (void)vault_name;
    (void)out_mask;
    return false;
#endif
}

// ── 마스킹 메타 사이드카 기록 ────────────────────────────────────────
//
//   사진에는 번호(F1, F2 …)가 눈에 보이게 새겨지지만, 그 번호가 화면
//   어디에 해당하는지는 사람이 눈으로 봐야 안다. 관리자 화면이나 서버가
//   기계적으로 다루려면 좌표가 필요하므로 같은 이름의 `.masks.json` 을
//   나란히 남긴다. 증거 사진 자체는 건드리지 않는다.
//
void CameraModule::WriteMaskSidecar(const std::string& image_path,
                                    const FaceMaskResult& mask) {
    if (!face_mask_params_.enable) return;
    if (image_path.empty()) return;

    const std::string side = image_path + ".masks.json";
    std::ofstream f(side);
    if (!f) return;

    f << "{\n";
    f << "  \"image\": \"" << std::filesystem::path(image_path).filename().string()
      << "\",\n";
    f << "  \"backend\": \"" << (mask.backend ? mask.backend : "none") << "\",\n";
    f << "  \"used_fallback\": " << (mask.used_fallback ? "true" : "false") << ",\n";
    f << "  \"face_count\": " << mask.faces << ",\n";
    f << "  \"label_prefix\": \"" << face_mask_params_.label_prefix << "\",\n";
    f << "  \"faces\": [";
    for (size_t i = 0; i < mask.boxes.size(); ++i) {
        const auto& b = mask.boxes[i];
        if (i) f << ",";
        f << "\n    {\"label\": \"" << face_mask_params_.label_prefix << b.index
          << "\", \"index\": " << b.index
          << ", \"x\": " << b.x << ", \"y\": " << b.y
          << ", \"w\": " << b.w << ", \"h\": " << b.h << "}";
    }
    f << (mask.boxes.empty() ? "" : "\n  ") << "]\n";
    f << "}\n";
}

bool CameraModule::Start() {
#ifdef USE_OPENCV
    if (running_) return true;

    // 얼굴 검출기는 무거우므로 캡처 시작 시 1회만 로드한다.
    if (!face_masker_) {
        face_masker_ = std::make_unique<FaceMasker>(face_mask_params_);
    }

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

bool CameraModule::RequestBlackboxSave(const std::string& path,
                                       const BlackboxClipMeta& meta) {
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
                                   path, NowEpochMs(), meta);
    return true;
#else
    (void)path;
    (void)meta;
    return false;
#endif
}

void CameraModule::BlackboxSaveWorker(std::string path, int64_t event_ms,
                                      BlackboxClipMeta meta) {
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

    // ── 얼굴 마스킹 (프라이버시 2계층) ───────────────────────────────
    //
    //   링버퍼에는 원본 프레임이 들어 있다 (10Hz 로 상시 인코딩하는데
    //   거기서 얼굴 검출까지 돌리면 스캔 루프에 부담이 크기 때문).
    //   그래서 **디스크에 쓰기 직전인 여기서** 프레임마다 마스킹한다.
    //   원본 JPEG 는 메모리에만 존재하고 파일로 남지 않는다.
    //
    //   검출기는 스레드 안전하지 않으므로 이 worker 전용 인스턴스를 쓴다
    //   (사진 경로의 face_masker_ 와 동시 호출될 수 있음).
    if (!blackbox_masker_) {
        blackbox_masker_ = std::make_unique<FaceMasker>(face_mask_params_);
    }
    {
        // fail-closed: 마스킹이 요구됐는데 불가능하면 영상을 저장하지 않는다.
        cv::Mat probe = first.clone();
        const FaceMaskResult pr = blackbox_masker_->Apply(&probe);
        if (!pr.safe_to_emit) {
            std::cerr << "[FACEMASK] fail-closed — 마스킹 불가로 블랙박스 영상 "
                         "저장을 건너뜁니다 (" << path << ")\n";
            blackbox_saving_ = false;
            return;
        }
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
    size_t masked_frames = 0;
    int    masked_faces  = 0;
    for (const auto& f : frames) {
        cv::Mat img = cv::imdecode(f.jpeg, cv::IMREAD_COLOR);
        if (img.empty()) continue;
        if (img.size() != first.size()) cv::resize(img, img, first.size());

        const FaceMaskResult mr = blackbox_masker_->Apply(&img);
        if (!mr.safe_to_emit) continue;   // 이 프레임은 버린다
        if (mr.applied) { masked_frames++; masked_faces += mr.faces; }

        writer.write(img);
        written++;
    }
    writer.release();

    std::cout << "[CAMERA] Blackbox clip saved: " << path
              << " (" << written << " frames, "
              << (end_ms - begin_ms) / 1000 << "s window)\n";
    if (face_mask_params_.enable) {
        std::cout << "[FACEMASK] 블랙박스 마스킹 — " << masked_frames << "/"
                  << written << " 프레임에서 얼굴 " << masked_faces
                  << "개 처리 (backend=" << blackbox_masker_->BackendName()
                  << ")\n";
    }

    // 저장 완료 통지 — 서버 업로드 등 후처리는 콜백 쪽 큐에서 비동기 수행.
    if (written > 0 && blackbox_saved_cb_) {
        blackbox_saved_cb_(path, meta);
    }
    blackbox_saving_ = false;
#else
    (void)path;
    (void)event_ms;
    (void)meta;
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

    // 서버로 나가는 이미지다 — 반드시 마스킹을 거친다.
    {
        std::ostringstream vn;
        vn << "b64_" << std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
        if (!PrepareEvidenceFrame(&frame, vn.str())) return "";
    }

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

    if (!PrepareEvidenceFrame(
            &frame, std::filesystem::path(filename).stem().string())) {
        return "";
    }

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    if (cv::imwrite(filename, frame, params)) {
        std::cout << "[CAMERA] Async snapshot saved: " << filename << "\n";
        return filename;
    }
#endif
    return "";
}

std::string CameraModule::CaptureToFilePath(const std::string& path,
                                            const std::string& annotation,
                                            const std::string& head_text,
                                            double head_offset_deg,
                                            double head_distance_mm) {
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

    // ★ 원본 vault 보관 + 얼굴 마스킹은 배너를 그리기 전에 끝낸다.
    //   (마스킹을 나중에 하면 ID 라벨까지 블러 처리된다)
    FaceMaskResult mask_info;
    if (!PrepareEvidenceFrame(&frame,
                              std::filesystem::path(path).stem().string(),
                              &mask_info)) {
        return "";
    }

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

    // 사람 머리 위 ID 라벨: LiDAR 좌표 기반 추정 위치에 라벨 + 포인터 라인.
    if (!head_text.empty()) {
        int hx = 0;
        int hy = 0;
        if (ComputeHeadLabelPos(head_label_, head_offset_deg, head_distance_mm,
                                frame.cols, frame.rows, &hx, &hy)) {
            constexpr int kPad = 6;
            constexpr double kScale = 0.9;
            constexpr int kThickness = 2;
            int baseline = 0;
            const cv::Size sz = cv::getTextSize(
                head_text, cv::FONT_HERSHEY_SIMPLEX, kScale, kThickness, &baseline);
            const int box_w = sz.width + kPad * 2;
            const int box_h = sz.height + baseline + kPad * 2;
            int left = hx - box_w / 2;
            int top = hy - box_h;
            left = std::max(0, std::min(left, frame.cols - box_w));
            top = std::max(0, std::min(top, frame.rows - box_h));
            const cv::Rect box(left, top, box_w, box_h);
            cv::rectangle(frame, box, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 1);
            cv::putText(frame, head_text,
                        cv::Point(box.x + kPad, box.y + kPad + sz.height),
                        cv::FONT_HERSHEY_SIMPLEX, kScale,
                        cv::Scalar(0, 255, 0), kThickness);
            // 라벨 아래 포인터 라인 — 어느 사람을 가리키는지 표시
            const int line_bottom =
                std::min(frame.rows - 1, box.y + box_h + 24);
            cv::line(frame, cv::Point(hx, box.y + box_h),
                     cv::Point(hx, line_bottom),
                     cv::Scalar(0, 255, 0), 2);
        }
    }

    const std::filesystem::path out_path(path);
    if (out_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);
    }

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    if (cv::imwrite(path, frame, params)) {
        std::cout << "[CAMERA] Snapshot saved: " << path << "\n";
        WriteMaskSidecar(path, mask_info);
        return path;
    }
#else
    (void)path;
    (void)annotation;
    (void)head_text;
    (void)head_offset_deg;
    (void)head_distance_mm;
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
