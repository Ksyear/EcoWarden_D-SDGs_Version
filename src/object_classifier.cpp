/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 객체 분류 구현 (DNN + LiDAR 기하 휴리스틱)
 */

#include "object_classifier.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#ifdef HAVE_OPENCV_DNN
#include <opencv2/dnn.hpp>
#endif
#endif

namespace ecowarden {

const char* ObjectClassToString(ObjectClass c) {
    switch (c) {
        case ObjectClass::kPerson:       return "person";
        case ObjectClass::kAnimal:       return "animal";
        case ObjectClass::kStaticObject: return "static_object";
        case ObjectClass::kNoise:        return "noise";
        case ObjectClass::kUnknown:
        default:                         return "unknown";
    }
}

const char* EventSeverityToString(EventSeverity s) {
    switch (s) {
        case EventSeverity::kHigh:   return "high";
        case EventSeverity::kMedium: return "medium";
        case EventSeverity::kLow:    return "low";
        case EventSeverity::kIgnore:
        default:                     return "ignore";
    }
}

const char* ClassifierBackendToString(ClassifierBackend b) {
    switch (b) {
        case ClassifierBackend::kDnn:       return "dnn";
        case ClassifierBackend::kLidarGeom: return "lidar_geom";
        case ClassifierBackend::kNone:
        default:                            return "none";
    }
}

const char* EventTypeForClass(ObjectClass c) {
    switch (c) {
        case ObjectClass::kPerson:       return "intrusion";
        case ObjectClass::kAnimal:       return "animal_or_small_object";
        case ObjectClass::kStaticObject: return "animal_or_small_object";
        case ObjectClass::kNoise:        return "noise";
        case ObjectClass::kUnknown:
        default:                         return "unknown_object";
    }
}

// ── 등급 산정 ────────────────────────────────────────────────────────
EventSeverity GradeEvent(const ClassifyResult& r, bool restricted_zone) {
    EventSeverity base;
    switch (r.cls) {
        case ObjectClass::kNoise:
            return EventSeverity::kIgnore;   // 금지구역이어도 올리지 않는다

        case ObjectClass::kPerson:
            // 신뢰도가 낮은 "사람"은 medium 으로 낮춰 사람이 확인하게 한다.
            base = (r.confidence >= 0.5) ? EventSeverity::kHigh
                                         : EventSeverity::kMedium;
            break;

        case ObjectClass::kAnimal:
        case ObjectClass::kStaticObject:
            base = EventSeverity::kLow;
            break;

        case ObjectClass::kUnknown:
        default:
            // ★ 모르면 올린다 — 미탐이 오탐보다 비싸다.
            base = EventSeverity::kMedium;
            break;
    }

    if (!restricted_zone) return base;

    // 금지구역 안이면 한 단계 상향
    switch (base) {
        case EventSeverity::kLow:    return EventSeverity::kMedium;
        case EventSeverity::kMedium: return EventSeverity::kHigh;
        default:                     return base;
    }
}

// ────────────────────────────────────────────────────────────────────
struct ObjectClassifier::Impl {
#if defined(USE_OPENCV) && defined(HAVE_OPENCV_DNN)
    cv::dnn::Net net;
#endif
    bool net_ok = false;
    std::vector<std::string> labels;
    const char* backend = "lidar_geom";
};

ObjectClassifier::ObjectClassifier(const ClassifierParams& params)
    : params_(params), impl_(std::make_unique<Impl>()) {
    if (!params_.enable) {
        impl_->backend = "none";
        return;
    }

#if defined(USE_OPENCV) && defined(HAVE_OPENCV_DNN)
    if (!params_.dnn_model.empty()) {
        try {
            impl_->net = params_.dnn_config.empty()
                             ? cv::dnn::readNet(params_.dnn_model)
                             : cv::dnn::readNet(params_.dnn_model,
                                                params_.dnn_config);
            if (!impl_->net.empty()) {
                impl_->net_ok = true;
                impl_->backend = "dnn";
                std::cout << "[CLASSIFY] DNN 백엔드 로드: "
                          << params_.dnn_model << "\n";
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[CLASSIFY] DNN 로드 실패 (" << e.what()
                      << ") — LiDAR 휴리스틱으로 폴백\n";
        }
    }
#endif

    if (!params_.dnn_labels.empty()) {
        std::ifstream in(params_.dnn_labels);
        std::string line;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) impl_->labels.push_back(line);
        }
    }

    if (!impl_->net_ok) {
        std::cout << "[CLASSIFY] LiDAR 기하 휴리스틱 사용 "
                     "(DNN 모델 미지정 또는 로드 실패 — 신뢰도 상한 "
                  << params_.lidar_confidence_cap << ")\n";
    }
}

ObjectClassifier::~ObjectClassifier() = default;

bool ObjectClassifier::DnnAvailable() const { return impl_->net_ok; }
const char* ObjectClassifier::BackendName() const { return impl_->backend; }

// ── LiDAR 기하 휴리스틱 ──────────────────────────────────────────────
//
//   2D LiDAR 로 확정할 수 있는 건 "폭·속도·지속성" 정도다. 그래서
//   이 함수는 **배제법**으로 동작한다: 명백히 노이즈/정지물체/동물인
//   것을 걸러내고, 사람 조건을 모두 만족할 때만 person 을 준다.
//   애매하면 unknown 을 돌려주고, 등급 단계에서 medium 으로 올라간다.
//
ClassifyResult ObjectClassifier::ClassifyFromLidar(
    const LidarFeatures& f) const {
    ClassifyResult r;
    r.backend = ClassifierBackend::kLidarGeom;

    const double cap = std::clamp(params_.lidar_confidence_cap, 0.1, 1.0);

    // ① 너무 짧게 관측 → 노이즈
    if (f.age_frames <= params_.noise_max_age_frames &&
        f.match_frames <= params_.noise_max_age_frames) {
        r.cls = ObjectClass::kNoise;
        r.confidence = cap * 0.9;
        r.reason = "관측 프레임 부족 (age=" + std::to_string(f.age_frames) + ")";
        return r;
    }

    // ② 트래커가 사람 그룹으로 묶었으면 그게 가장 강한 근거
    if (f.in_person_group) {
        r.cls = ObjectClass::kPerson;
        r.confidence = cap;
        r.reason = "트래커 PersonGroup 소속";
        return r;
    }

    // ③ 오래 정지 + 이동 이력 없음 → 정지 물체 (투기물/가구)
    if (f.stationary_frames >= params_.static_min_stationary_frames &&
        !f.was_moving) {
        r.cls = ObjectClass::kStaticObject;
        r.confidence = cap * 0.85;
        r.reason = "장시간 정지 + 이동 이력 없음";
        return r;
    }

    const bool width_person =
        f.width_mm >= params_.person_min_width_mm &&
        f.width_mm <= params_.person_max_width_mm;
    const bool speed_person =
        f.speed_mm_s >= params_.person_min_speed_mm_s &&
        f.speed_mm_s <= params_.person_max_speed_mm_s;
    const bool enough_match = f.match_frames >= params_.person_min_match_frames;

    // ── ④·⑤ 동물 판정 — 겹치는 대역을 사람 쪽으로 양보한다 ─────────
    //
    //   중요: 사람 폭 대역 [150,900]mm 은 동물 대역 [0,450]mm 을 거의
    //   포함한다. 15cm 높이에서 잡히는 중형견 단면(~250mm)과 사람 다리
    //   하나(~150mm)는 2D LiDAR 로 구분되지 않는다.
    //
    //   그래서 겹치는 구간은 **동물로 내리지 않는다**. 동물로 잘못
    //   내리면 severity 가 low 로 떨어져 진짜 사람을 놓치기 때문이다
    //   (미탐 > 오탐 이라는 프로젝트 전제와 정반대). 동물 판정은 아래
    //   두 경우, 즉 "사람 단면일 수 없는" 근거가 있을 때만 한다.

    // ④ 사람 최소 폭보다도 좁고 움직인다 → 동물
    if (f.width_mm > 0.0 && f.width_mm < params_.person_min_width_mm &&
        f.speed_mm_s >= params_.animal_min_speed_mm_s) {
        r.cls = ObjectClass::kAnimal;
        r.confidence = cap * 0.6;   // 동물 판정은 특히 약한 근거다
        r.reason = "폭 " + std::to_string(static_cast<int>(f.width_mm)) +
                   "mm < 사람 최소폭 " +
                   std::to_string(static_cast<int>(params_.person_min_width_mm)) +
                   "mm, 이동 중";
        return r;
    }

    // ⑤ 보행 속도 상한을 넘는 질주 + 좁은 단면 → 동물
    if (f.width_mm > 0.0 && f.width_mm <= params_.animal_max_width_mm &&
        f.speed_mm_s > params_.person_max_speed_mm_s) {
        r.cls = ObjectClass::kAnimal;
        r.confidence = cap * 0.5;   // 사람이 뛰어도 나올 수 있어 더 약하다
        r.reason = "속도 " + std::to_string(static_cast<int>(f.speed_mm_s)) +
                   "mm/s > 보행 상한, 좁은 단면";
        return r;
    }

    // ⑥ 폭·속도·지속성이 모두 사람 대역 → 사람
    //    (동물 대역과 겹치더라도 여기로 온다 — 안전한 방향)
    if (width_person && speed_person && enough_match) {
        r.cls = ObjectClass::kPerson;
        r.confidence = cap * 0.9;
        r.reason = "폭·속도·지속성 모두 사람 대역";
        return r;
    }

    // ⑦ 그 외 — 모른다. 등급 단계에서 medium 으로 올라간다.
    r.cls = ObjectClass::kUnknown;
    r.confidence = 0.3;
    r.reason = "사람/동물 조건 불충족 (폭=" +
               std::to_string(static_cast<int>(f.width_mm)) + "mm, 속도=" +
               std::to_string(static_cast<int>(f.speed_mm_s)) + "mm/s)";
    return r;
}

// ── DNN 백엔드 ───────────────────────────────────────────────────────
ClassifyResult ObjectClassifier::ClassifyFromFrame(
    void* frame_mat, const LidarFeatures& lidar_fallback) const {
#if defined(USE_OPENCV) && defined(HAVE_OPENCV_DNN)
    if (impl_->net_ok && frame_mat != nullptr) {
        cv::Mat& frame = *static_cast<cv::Mat*>(frame_mat);
        if (!frame.empty()) {
            try {
                const int sz = params_.dnn_input_size;
                cv::Mat blob = cv::dnn::blobFromImage(
                    frame, params_.dnn_scale, cv::Size(sz, sz),
                    cv::Scalar(params_.dnn_mean, params_.dnn_mean,
                               params_.dnn_mean),
                    true, false);
                impl_->net.setInput(blob);
                cv::Mat out = impl_->net.forward();

                // SSD 계열 출력 [1,1,N,7] 을 우선 처리
                if (out.dims == 4 && out.size[3] == 7) {
                    cv::Mat d(out.size[2], out.size[3], CV_32F,
                              out.ptr<float>());
                    double best = 0.0;
                    int best_label = -1;
                    for (int i = 0; i < d.rows; ++i) {
                        const float conf = d.at<float>(i, 2);
                        if (conf < params_.dnn_confidence) continue;
                        if (conf > best) {
                            best = conf;
                            best_label = static_cast<int>(d.at<float>(i, 1));
                        }
                    }
                    if (best_label >= 0) {
                        ClassifyResult r;
                        r.backend = ClassifierBackend::kDnn;
                        r.confidence = best;
                        std::string name =
                            (best_label < static_cast<int>(impl_->labels.size()))
                                ? impl_->labels[best_label]
                                : std::to_string(best_label);
                        for (char& ch : name) ch = static_cast<char>(::tolower(ch));

                        if (name.find("person") != std::string::npos ||
                            name.find("people") != std::string::npos) {
                            r.cls = ObjectClass::kPerson;
                        } else if (name.find("cat") != std::string::npos ||
                                   name.find("dog") != std::string::npos ||
                                   name.find("bird") != std::string::npos ||
                                   name.find("animal") != std::string::npos ||
                                   name.find("horse") != std::string::npos ||
                                   name.find("sheep") != std::string::npos ||
                                   name.find("cow") != std::string::npos) {
                            r.cls = ObjectClass::kAnimal;
                        } else {
                            r.cls = ObjectClass::kUnknown;
                        }
                        r.reason = "DNN label=" + name;
                        return r;
                    }
                }
            } catch (const cv::Exception& e) {
                std::cerr << "[CLASSIFY] DNN 추론 실패: " << e.what()
                          << " — LiDAR 폴백\n";
            }
        }
    }
#else
    (void)frame_mat;
#endif
    // DNN 이 없거나 확신 못 하면 LiDAR 휴리스틱으로.
    return ClassifyFromLidar(lidar_fallback);
}

} // namespace ecowarden
