/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 증거 사진 얼굴 자동 마스킹 (3계층 프라이버시 2계층)
 */

/**
 * @file  face_masking.h
 * @brief 증거 사진에서 얼굴을 검출해 블러/모자이크 처리한다.
 *
 * 프라이버시 3계층 구조에서 **2계층(공개본)**을 담당한다.
 *
 *   1계층 평상시     : 카메라 미작동, 비식별 LiDAR 좌표만        (기존 구현)
 *   2계층 공개본     : 얼굴 마스킹본만 저장·전송                 ← 이 모듈
 *   3계층 원본       : 암호화 보관 + 보존기한 후 자동 파기        (evidence_vault.h)
 *
 * 설계 원칙 — **fail-closed**:
 *   얼굴 검출기를 로드하지 못했는데 마스킹이 켜져 있으면, 아무 처리도
 *   안 한 원본을 내보내지 않는다. `fallback_mask_upper` 가 켜져 있으면
 *   화면 상단(사람 머리가 들어오는 영역)을 통째로 마스킹하고, 그것도
 *   꺼져 있으면 저장 자체를 거부한다. "마스킹한다고 말해 놓고 실제로는
 *   원본이 나가는" 상황을 구조적으로 막기 위한 것이다.
 *
 * 백엔드 (우선순위 순):
 *   1. DNN (OpenCV dnn, res10 SSD 등) — `dnn_config` + `dnn_weights` 지정 시
 *   2. Haar cascade — OpenCV 기본 제공 `haarcascade_frontalface_default.xml`
 *   3. (실패) fail-closed 폴백
 *
 * OpenCV 없이 빌드하면 이 모듈은 아무 것도 하지 않는 스텁이 되고,
 * `Available()` 은 false 를 돌려준다 — 이때 `require_detector` 가 켜져
 * 있으면 CameraModule 이 사진 저장을 건너뛴다.
 */

#pragma once

#include <memory>
#include <string>

namespace ecowarden {

// ── 마스킹 방식 ──────────────────────────────────────────────────────
enum class FaceMaskMode {
    kBlur = 0,      // 가우시안 블러 (기본) — 되돌릴 수 없는 강도로 적용
    kPixelate,      // 모자이크 — 시각적으로 "가렸다"는 게 더 명확
    kBox,           // 검은 박스 — 가장 강함, 증거 사진 판독성은 떨어짐
};

FaceMaskMode ParseFaceMaskMode(const std::string& s);
const char*  FaceMaskModeToString(FaceMaskMode m);

// ── 파라미터 ─────────────────────────────────────────────────────────
struct FaceMaskParams {
    bool enable = true;

    FaceMaskMode mode = FaceMaskMode::kBlur;

    // 검출기 경로. 비워 두면 OpenCV 설치 경로에서 자동 탐색한다.
    std::string cascade_path;

    // 선택: DNN 백엔드 (둘 다 지정해야 사용). Haar 보다 정확하지만 무겁다.
    std::string dnn_config;    // .prototxt / .cfg
    std::string dnn_weights;   // .caffemodel / .onnx / .weights
    double      dnn_confidence = 0.5;

    // Haar 검출 파라미터
    double scale_factor  = 1.1;
    int    min_neighbors = 5;
    int    min_face_px   = 24;   // 이보다 작은 얼굴은 무시 (노이즈)

    // 검출 박스를 얼마나 넓혀 마스킹할지 — 머리카락·턱선까지 덮기 위함
    double expand_ratio = 0.25;

    // kBlur: ROI 짧은 변 대비 커널 비율 (0.35 ≈ 형체만 남는 수준)
    double blur_strength = 0.35;
    // kPixelate: ROI 를 몇 블록으로 나눌지 (작을수록 강함)
    int    pixelate_blocks = 10;

    // ── fail-closed ──────────────────────────────────────────────────
    // 검출기 로드 실패 시 원본을 그대로 내보내지 않는다.
    bool   require_detector    = true;
    // 검출기가 없을 때 화면 상단을 통째로 마스킹할지
    bool   fallback_mask_upper = true;
    // 폴백 마스킹 영역 (화면 높이 대비 비율)
    double fallback_upper_ratio = 0.45;
};

// ── 마스킹 결과 ──────────────────────────────────────────────────────
struct FaceMaskResult {
    bool        applied     = false;  // 실제로 마스킹이 적용됐는가
    int         faces       = 0;      // 검출·마스킹한 얼굴 수
    bool        used_fallback = false;// 폴백(상단 통째) 경로였는가
    bool        safe_to_emit = false; // 이 프레임을 내보내도 되는가
    const char* backend     = "none"; // "dnn" | "haar" | "fallback" | "none"
};

/**
 * @brief 얼굴 검출기를 1회 로드해 재사용하는 마스커.
 *
 *   OpenCV 타입을 public header 에 노출하지 않기 위해 PIMPL 을 쓴다
 *   (CameraModule 과 같은 방식). 프레임은 `cv::Mat*` 를 void* 로 받는다 —
 *   내부 전용 인터페이스이므로 호출부는 camera_module.cpp 하나뿐이다.
 */
class FaceMasker {
public:
    explicit FaceMasker(const FaceMaskParams& params = {});
    ~FaceMasker();

    FaceMasker(const FaceMasker&) = delete;
    FaceMasker& operator=(const FaceMasker&) = delete;

    // 검출기가 실제로 로드됐는가 (OpenCV 미사용 빌드면 항상 false)
    bool Available() const;

    // 사용 중인 백엔드 이름
    const char* BackendName() const;

    /**
     * @brief 프레임의 얼굴을 in-place 로 마스킹한다.
     * @param frame_mat `cv::Mat*` (BGR). nullptr 이면 no-op.
     * @return 적용 결과. `safe_to_emit == false` 면 저장하면 안 된다.
     *
     *   주의: 반드시 배너·ID 라벨을 **그리기 전에** 호출해야 한다.
     *   그렇지 않으면 라벨까지 블러 처리된다.
     */
    FaceMaskResult Apply(void* frame_mat);

    const FaceMaskParams& params() const { return params_; }

private:
    FaceMaskParams params_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ecowarden
