/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 증거 사진 얼굴 자동 마스킹 구현
 */

#include "face_masking.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#ifdef HAVE_OPENCV_DNN
#include <opencv2/dnn.hpp>
#endif
#include <filesystem>
#endif

namespace ecowarden {

FaceMaskMode ParseFaceMaskMode(const std::string& s) {
    if (s == "pixelate" || s == "mosaic") return FaceMaskMode::kPixelate;
    if (s == "box" || s == "black")       return FaceMaskMode::kBox;
    return FaceMaskMode::kBlur;
}

const char* FaceMaskModeToString(FaceMaskMode m) {
    switch (m) {
        case FaceMaskMode::kPixelate: return "pixelate";
        case FaceMaskMode::kBox:      return "box";
        case FaceMaskMode::kBlur:
        default:                      return "blur";
    }
}

// ────────────────────────────────────────────────────────────────────
#ifdef USE_OPENCV

namespace {

// OpenCV 가 설치 경로에 함께 깔아 두는 Haar cascade 를 찾는다.
// 배포판마다 위치가 달라 후보를 순서대로 훑는다.
std::string FindDefaultCascade() {
    static const char* kCandidates[] = {
        // Debian/Ubuntu (Raspberry Pi OS 포함)
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        // Homebrew (Apple Silicon / Intel)
        "/opt/homebrew/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        // 저장소 동봉 (setup.sh 가 내려받는 경로)
        "models/haarcascade_frontalface_default.xml",
        "./haarcascade_frontalface_default.xml",
    };
    std::error_code ec;
    for (const char* p : kCandidates) {
        if (std::filesystem::exists(p, ec)) return p;
    }
    return {};
}

// 커널 크기는 홀수여야 한다.
int OddAtLeast3(int v) {
    if (v < 3) return 3;
    return (v % 2 == 0) ? v + 1 : v;
}

void MaskRoi(cv::Mat& frame, const cv::Rect& roi_in, const FaceMaskParams& p) {
    const cv::Rect bounds(0, 0, frame.cols, frame.rows);
    const cv::Rect roi = roi_in & bounds;
    if (roi.width <= 1 || roi.height <= 1) return;

    cv::Mat region = frame(roi);

    switch (p.mode) {
        case FaceMaskMode::kBox:
            region.setTo(cv::Scalar(0, 0, 0));
            break;

        case FaceMaskMode::kPixelate: {
            const int blocks = std::max(2, p.pixelate_blocks);
            const int w = std::max(1, roi.width  / blocks);
            const int h = std::max(1, roi.height / blocks);
            cv::Mat small;
            cv::resize(region, small, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
            cv::resize(small, region, region.size(), 0, 0, cv::INTER_NEAREST);
            break;
        }

        case FaceMaskMode::kBlur:
        default: {
            const int shorter = std::min(roi.width, roi.height);
            const int k = OddAtLeast3(
                static_cast<int>(shorter * std::max(0.05, p.blur_strength)));
            // sigma 를 0 으로 두면 커널 크기에서 자동 산출된다.
            cv::GaussianBlur(region, region, cv::Size(k, k), 0);
            // 한 번 더 — 얼굴 윤곽 복원 시도를 어렵게 한다.
            cv::GaussianBlur(region, region, cv::Size(k, k), 0);
            break;
        }
    }
}

cv::Rect ExpandRect(const cv::Rect& r, double ratio) {
    const int dx = static_cast<int>(r.width  * ratio);
    const int dy = static_cast<int>(r.height * ratio);
    return cv::Rect(r.x - dx, r.y - dy, r.width + dx * 2, r.height + dy * 2);
}

// 마스킹한 자리에 번호를 새긴다.
//
//   얼굴을 가리면 사진만으로 사람을 구별할 수 없다. 관리자가 "1번이
//   버렸다" 고 지목할 수 있어야 증거로서 의미가 있으므로, 마스킹 영역
//   위에 번호를 그린다. 번호는 마스킹 **후에** 그려야 지워지지 않는다.
void DrawFaceLabel(cv::Mat& frame, const cv::Rect& roi, int index,
                   const FaceMaskParams& p) {
    if (!p.label_faces) return;
    const cv::Rect bounds(0, 0, frame.cols, frame.rows);
    const cv::Rect box = roi & bounds;
    if (box.width <= 1 || box.height <= 1) return;

    const std::string text = p.label_prefix + std::to_string(index);
    const double scale = std::max(0.4, p.label_scale);
    constexpr int kThick = 2;
    int baseline = 0;
    const cv::Size sz =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, kThick, &baseline);

    // 마스킹 영역 중앙에 배치 (박스 밖으로 나가지 않게 클램프)
    int tx = box.x + (box.width - sz.width) / 2;
    int ty = box.y + (box.height + sz.height) / 2;
    tx = std::max(0, std::min(tx, frame.cols - sz.width));
    ty = std::max(sz.height, std::min(ty, frame.rows - baseline));

    // 마스킹 영역 테두리 — 어디를 가렸는지 보이게 한다
    cv::rectangle(frame, box, cv::Scalar(0, 200, 255), 2);
    // 글자 가독성용 배경
    cv::rectangle(frame,
                  cv::Point(tx - 4, ty - sz.height - 4),
                  cv::Point(tx + sz.width + 4, ty + baseline),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(frame, text, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
                scale, cv::Scalar(0, 200, 255), kThick);
}

} // namespace

struct FaceMasker::Impl {
    cv::CascadeClassifier cascade;
    bool cascade_ok = false;
#ifdef HAVE_OPENCV_DNN
    cv::dnn::Net net;
    bool net_ok = false;
#endif
    const char* backend = "none";
};

FaceMasker::FaceMasker(const FaceMaskParams& params)
    : params_(params), impl_(std::make_unique<Impl>()) {
    if (!params_.enable) return;

#ifdef HAVE_OPENCV_DNN
    // 1순위: DNN 백엔드 (둘 다 지정된 경우에만)
    if (!params_.dnn_config.empty() && !params_.dnn_weights.empty()) {
        try {
            impl_->net = cv::dnn::readNet(params_.dnn_weights, params_.dnn_config);
            if (!impl_->net.empty()) {
                impl_->net_ok = true;
                impl_->backend = "dnn";
                std::cout << "[FACEMASK] DNN backend loaded: "
                          << params_.dnn_weights << "\n";
                return;
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[FACEMASK] DNN load failed (" << e.what()
                      << ") — Haar 로 폴백\n";
        }
    }
#endif

    // 2순위: Haar cascade
    std::string path = params_.cascade_path;
    if (path.empty()) path = FindDefaultCascade();
    if (!path.empty() && impl_->cascade.load(path)) {
        impl_->cascade_ok = true;
        impl_->backend = "haar";
        std::cout << "[FACEMASK] Haar cascade loaded: " << path << "\n";
        return;
    }

    std::cerr << "[FACEMASK] 얼굴 검출기를 로드하지 못했습니다"
              << (path.empty() ? " (cascade 파일 미발견)" : (" (" + path + ")"))
              << ".\n";
    if (params_.require_detector) {
        std::cerr << "[FACEMASK] fail-closed 모드 — "
                  << (params_.fallback_mask_upper
                          ? "상단 영역을 통째로 마스킹합니다.\n"
                          : "증거 사진 저장을 건너뜁니다.\n");
    } else {
        std::cerr << "[FACEMASK] 경고: 마스킹 없이 원본이 저장됩니다 "
                     "(ECOWARDEN_FACE_MASK_REQUIRE=1 권장).\n";
    }
}

FaceMasker::~FaceMasker() = default;

bool FaceMasker::Available() const {
#ifdef HAVE_OPENCV_DNN
    if (impl_->net_ok) return true;
#endif
    return impl_->cascade_ok;
}

const char* FaceMasker::BackendName() const { return impl_->backend; }

FaceMaskResult FaceMasker::Apply(void* frame_mat) {
    FaceMaskResult r;
    r.backend = impl_->backend;

    if (!params_.enable) {
        // 마스킹을 명시적으로 끈 경우 — 저장은 허용한다(운영자 선택).
        r.safe_to_emit = true;
        return r;
    }
    if (frame_mat == nullptr) return r;

    cv::Mat& frame = *static_cast<cv::Mat*>(frame_mat);
    if (frame.empty()) return r;

    std::vector<cv::Rect> faces;

#ifdef HAVE_OPENCV_DNN
    if (impl_->net_ok) {
        try {
            cv::Mat blob = cv::dnn::blobFromImage(
                frame, 1.0, cv::Size(300, 300),
                cv::Scalar(104.0, 177.0, 123.0), false, false);
            impl_->net.setInput(blob);
            cv::Mat det = impl_->net.forward();
            cv::Mat d(det.size[2], det.size[3], CV_32F, det.ptr<float>());
            for (int i = 0; i < d.rows; ++i) {
                const float conf = d.at<float>(i, 2);
                if (conf < params_.dnn_confidence) continue;
                const int x1 = static_cast<int>(d.at<float>(i, 3) * frame.cols);
                const int y1 = static_cast<int>(d.at<float>(i, 4) * frame.rows);
                const int x2 = static_cast<int>(d.at<float>(i, 5) * frame.cols);
                const int y2 = static_cast<int>(d.at<float>(i, 6) * frame.rows);
                cv::Rect box(cv::Point(x1, y1), cv::Point(x2, y2));
                if (box.width >= params_.min_face_px &&
                    box.height >= params_.min_face_px) {
                    faces.push_back(box);
                }
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[FACEMASK] DNN 추론 실패: " << e.what() << "\n";
            faces.clear();
        }
    } else
#endif
    if (impl_->cascade_ok) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 축소본에서 검출하고 박스만 원본 좌표로 되돌린다 (마스킹은 원본에).
        double inv = 1.0;
        if (params_.detect_max_width > 0 &&
            gray.cols > params_.detect_max_width) {
            const double f =
                static_cast<double>(params_.detect_max_width) / gray.cols;
            cv::resize(gray, gray, cv::Size(), f, f, cv::INTER_AREA);
            inv = 1.0 / f;
        }
        cv::equalizeHist(gray, gray);

        // 최소 얼굴 크기도 같은 배율로 줄여야 기준이 유지된다.
        const int min_px = std::max(
            8, static_cast<int>(params_.min_face_px / inv));
        try {
            impl_->cascade.detectMultiScale(
                gray, faces, params_.scale_factor, params_.min_neighbors, 0,
                cv::Size(min_px, min_px));
        } catch (const cv::Exception& e) {
            std::cerr << "[FACEMASK] Haar 검출 실패: " << e.what() << "\n";
            faces.clear();
        }
        if (inv != 1.0) {
            for (cv::Rect& f : faces) {
                f = cv::Rect(static_cast<int>(f.x * inv),
                             static_cast<int>(f.y * inv),
                             static_cast<int>(f.width * inv),
                             static_cast<int>(f.height * inv));
            }
        }
    } else {
        // ── fail-closed 경로 ─────────────────────────────────────────
        if (!params_.require_detector) {
            r.safe_to_emit = true;   // 운영자가 명시적으로 허용
            return r;
        }
        if (params_.fallback_mask_upper) {
            const int h = static_cast<int>(
                frame.rows * std::clamp(params_.fallback_upper_ratio, 0.1, 1.0));
            const cv::Rect roi(0, 0, frame.cols, h);
            MaskRoi(frame, roi, params_);
            r.applied       = true;
            r.used_fallback = true;
            r.safe_to_emit  = true;
            r.backend       = "fallback";
            r.boxes.push_back({0, roi.x, roi.y, roi.width, roi.height});
            return r;
        }
        // 폴백조차 꺼져 있으면 내보내지 않는다.
        r.safe_to_emit = false;
        return r;
    }

    // 번호가 프레임마다 흔들리지 않도록 화면 왼쪽부터 정렬해 매긴다.
    std::sort(faces.begin(), faces.end(),
              [](const cv::Rect& a, const cv::Rect& b) { return a.x < b.x; });

    int idx = 0;
    for (const cv::Rect& f : faces) {
        const cv::Rect roi = ExpandRect(f, params_.expand_ratio);
        MaskRoi(frame, roi, params_);            // 먼저 가리고
        DrawFaceLabel(frame, roi, ++idx, params_); // 그 위에 번호를 새긴다
        const cv::Rect clipped = roi & cv::Rect(0, 0, frame.cols, frame.rows);
        r.boxes.push_back({idx, clipped.x, clipped.y,
                           clipped.width, clipped.height});
    }
    r.faces        = static_cast<int>(faces.size());
    r.applied      = !faces.empty();
    r.safe_to_emit = true;   // 검출기가 정상 동작했으므로 내보내도 된다
    return r;
}

#else // ── OpenCV 미사용 빌드 (Mac 스텁 등) ─────────────────────────

struct FaceMasker::Impl {};

FaceMasker::FaceMasker(const FaceMaskParams& params)
    : params_(params), impl_(std::make_unique<Impl>()) {}
FaceMasker::~FaceMasker() = default;

bool FaceMasker::Available() const { return false; }
const char* FaceMasker::BackendName() const { return "none"; }

FaceMaskResult FaceMasker::Apply(void* frame_mat) {
    (void)frame_mat;
    FaceMaskResult r;
    // OpenCV 가 없으면 애초에 프레임도 없다. 저장 경로 자체가 no-op 이므로
    // safe_to_emit 는 "마스킹 요구가 없을 때만" true.
    r.safe_to_emit = !params_.enable || !params_.require_detector;
    return r;
}

#endif // USE_OPENCV

} // namespace ecowarden
