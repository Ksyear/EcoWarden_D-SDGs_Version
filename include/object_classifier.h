/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 객체 분류 (사람 / 동물 / 불명) 및 이벤트 등급 산정
 */

/**
 * @file  object_classifier.h
 * @brief 이벤트 객체를 사람·동물·불명으로 분류하고 severity 등급을 매긴다.
 *
 * `보안/전체_구조.md` §5·§6 이 스펙만 정의하고 구현이 비어 있던 부분이다.
 *
 * ── 왜 2단 백엔드인가 ────────────────────────────────────────────────
 *
 *   2D LiDAR 는 객체 **종류**를 확정하는 센서가 아니다. 그래서 분류를
 *   LiDAR 단독으로 하면 과장이 된다. 반대로 카메라 DNN 만 쓰면 모델
 *   파일이 없거나 OpenCV dnn 이 빠진 빌드에서 기능 전체가 죽는다.
 *
 *   그래서 두 백엔드를 두고, **둘의 신뢰도를 구분해서 보고**한다.
 *
 *     1. kDnn        — 카메라 영상 기반. 모델 파일이 있을 때만.  (높은 신뢰)
 *     2. kLidarGeom  — LiDAR 폭·속도·지속성 기반 휴리스틱.        (낮은 신뢰)
 *
 *   DNN 이 없으면 kLidarGeom 이 동작하되, 결과에 붙는 confidence 가
 *   낮게 나오고 등급도 보수적으로(=사람일 가능성을 열어두고) 매겨진다.
 *   "모르면 medium 으로 올려서 사람이 보게 한다" 가 원칙 — 미탐을 줄이는
 *   기존 설계 철학과 같다.
 *
 * ── 등급 체계 (`전체_구조.md` §6) ────────────────────────────────────
 *
 *   | 분류            | 조건                        | severity |
 *   |-----------------|-----------------------------|----------|
 *   | person          | 사람 확정                    | high     |
 *   | unknown         | 분류 불확실                  | medium   |
 *   | animal / small  | 소형 동물·짧은 움직임         | low      |
 *   | noise           | 반사·센서 이상               | ignore   |
 *
 *   금지구역 안에서는 한 단계 올린다 (low→medium, medium→high).
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace ecowarden {

// ── 분류 결과 ────────────────────────────────────────────────────────
enum class ObjectClass {
    kUnknown = 0,   // 분류 불확실 — 사람일 가능성을 배제하지 않는다
    kPerson,
    kAnimal,        // 소형 동물 추정
    kStaticObject,  // 정지 물체 (투기물 포함)
    kNoise,         // 반사·스파이크 — 이벤트로 올리지 않음
};

const char* ObjectClassToString(ObjectClass c);

// ── 이벤트 등급 ──────────────────────────────────────────────────────
enum class EventSeverity {
    kIgnore = 0,
    kLow,
    kMedium,
    kHigh,
};

const char* EventSeverityToString(EventSeverity s);

// ── 어느 백엔드가 판단했는가 ─────────────────────────────────────────
enum class ClassifierBackend {
    kNone = 0,
    kLidarGeom,   // LiDAR 기하 휴리스틱
    kDnn,         // 카메라 DNN
};

const char* ClassifierBackendToString(ClassifierBackend b);

// ── LiDAR 트랙에서 뽑은 기하 특징 ────────────────────────────────────
struct LidarFeatures {
    double   width_mm         = 0.0;  // 클러스터 폭
    double   speed_mm_s       = 0.0;  // 프레임 간 속도
    uint32_t age_frames       = 0;    // 총 생존 프레임
    uint32_t match_frames     = 0;    // 누적 매칭 성공 프레임
    uint32_t stationary_frames= 0;    // 연속 정지 프레임
    bool     in_person_group  = false;// 트래커가 사람 그룹으로 묶었는가
    bool     was_moving       = false;// 정지 전 이동 이력
};

// ── 분류 결과 ────────────────────────────────────────────────────────
struct ClassifyResult {
    ObjectClass       cls        = ObjectClass::kUnknown;
    double            confidence = 0.0;   // 0.0 ~ 1.0
    ClassifierBackend backend    = ClassifierBackend::kNone;
    std::string       reason;             // 로그·디버그용 판단 근거
};

// ── 파라미터 ─────────────────────────────────────────────────────────
struct ClassifierParams {
    bool enable = true;

    // ── DNN 백엔드 (선택) ────────────────────────────────────────────
    std::string dnn_model;    // .onnx / .caffemodel / .weights
    std::string dnn_config;   // .prototxt / .cfg  (onnx 는 비워도 됨)
    std::string dnn_labels;   // 클래스 라벨 파일 (줄당 1개)
    double      dnn_confidence = 0.5;
    int         dnn_input_size = 300;
    double      dnn_scale      = 1.0 / 127.5;
    double      dnn_mean       = 127.5;

    // ── LiDAR 기하 휴리스틱 ──────────────────────────────────────────
    // 사람: 어깨/다리 폭. 15cm 설치 기준 다리 단면이 잡힌다.
    double person_min_width_mm = 150.0;
    double person_max_width_mm = 900.0;
    // 사람 보행 속도 대역 (mm/s). 0.3~2.2 m/s.
    double person_min_speed_mm_s = 300.0;
    double person_max_speed_mm_s = 2200.0;

    // 동물: 사람보다 좁고, 자세가 낮아 폭이 작게 잡힌다.
    double animal_max_width_mm = 450.0;
    // 동물은 순간 가속이 크다 — 사람 상한을 넘는 속도는 동물 쪽 근거.
    double animal_min_speed_mm_s = 400.0;

    // 이보다 짧게 관측된 트랙은 노이즈로 본다.
    uint32_t noise_max_age_frames = 3;
    // 사람으로 확정하려면 최소 이만큼 매칭돼야 한다.
    uint32_t person_min_match_frames = 5;
    // 이만큼 연속 정지면 정지 물체로 본다.
    uint32_t static_min_stationary_frames = 10;

    // 휴리스틱 결과에 곱하는 신뢰도 상한 — LiDAR 단독 판단은
    // 아무리 확실해 보여도 이 값을 넘지 않는다 (과장 방지).
    double lidar_confidence_cap = 0.7;
};

/**
 * @brief 사람/동물/불명 분류기.
 *
 *   DNN 모델이 지정되어 있고 로드에 성공하면 카메라 프레임으로 분류하고,
 *   그렇지 않으면 LiDAR 기하 특징으로 분류한다.
 */
class ObjectClassifier {
public:
    explicit ObjectClassifier(const ClassifierParams& params = {});
    ~ObjectClassifier();

    ObjectClassifier(const ObjectClassifier&) = delete;
    ObjectClassifier& operator=(const ObjectClassifier&) = delete;

    // DNN 백엔드가 실제로 로드됐는가
    bool DnnAvailable() const;
    const char* BackendName() const;

    /**
     * @brief LiDAR 기하 특징만으로 분류한다. 항상 사용 가능.
     *
     *   순수 함수라 단위 테스트가 쉽다 (test/sim_main.cpp 의 OC 시나리오).
     */
    ClassifyResult ClassifyFromLidar(const LidarFeatures& f) const;

    /**
     * @brief 카메라 프레임으로 분류한다 (DNN 백엔드).
     * @param frame_mat `cv::Mat*` (BGR). nullptr 이거나 DNN 미탑재면
     *        `lidar_fallback` 으로 자동 폴백한다.
     */
    ClassifyResult ClassifyFromFrame(void* frame_mat,
                                     const LidarFeatures& lidar_fallback) const;

    const ClassifierParams& params() const { return params_; }

private:
    ClassifierParams params_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ── 등급 산정 ────────────────────────────────────────────────────────
/**
 * @brief 분류 결과 + 구역 정책으로 이벤트 등급을 정한다.
 *
 * @param r               분류 결과
 * @param restricted_zone 금지구역 안에서 발생했는가 (한 단계 상향)
 *
 *   원칙: **모르면 올린다.** unknown 은 low 가 아니라 medium 이다.
 *   미탐(진짜 사건을 놓침)이 오탐보다 비싸다는 프로젝트 전제와 같다.
 */
EventSeverity GradeEvent(const ClassifyResult& r, bool restricted_zone);

/**
 * @brief 등급에 대응하는 이벤트 타입 문자열 (`전체_구조.md` §6).
 *   "intrusion" | "unknown_object" | "animal_or_small_object" | "noise"
 */
const char* EventTypeForClass(ObjectClass c);

} // namespace ecowarden
