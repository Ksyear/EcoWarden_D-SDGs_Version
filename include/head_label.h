/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 증거 사진 머리 위 ID 라벨 위치 계산
 */

/**
 * @file  head_label.h
 * @brief 증거 사진에서 "사람 머리 위" ID 라벨의 픽셀 좌표를 계산한다.
 * @date  2026
 *
 * OpenCV 에 의존하지 않는 순수 기하 계산이므로, OpenCV 가 없는 개발
 * 환경(Mac 스텁 빌드)에서도 이 로직만은 단위 검증이 가능하다.
 *
 * 기하 모델 (pin-hole, 카메라 pitch 0 가정):
 *   - 수평: 서보는 존 중심각으로 조준하므로, 사람의 실제 각도와 존
 *     중심각의 차(offset_deg)가 화면 중심 대비 수평 변위가 된다.
 *   - 수직: LiDAR 좌표의 거리와 (머리높이 - 카메라높이)로 고도각을
 *     계산해 화면 세로 위치를 추정한다. 2D LiDAR 는 높이를 모르므로
 *     머리 높이는 파라미터(기본 1.6m)로 가정한다.
 *   - 결과는 근사치다. 현장에서 라벨이 어긋나면 ECOWARDEN_CAMERA_HFOV_DEG
 *     등으로 보정하고, 좌우가 반대면 ECOWARDEN_HEAD_LABEL_IMG_MIRROR=1.
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace ecowarden {

// ── 머리 위 ID 라벨 파라미터 (env 연결은 production_params.h) ───────
struct HeadLabelParams {
    bool   enable        = true;   // 머리 위 ID 라벨 on/off
    double hfov_deg      = 62.0;   // 카메라 수평 화각
    double vfov_deg      = 38.0;   // 카메라 수직 화각
    double cam_height_m  = 0.15;   // 카메라 설치 높이 (LiDAR 마운트 기준)
    double head_height_m = 1.60;   // 가정하는 사람 머리 높이
    // 카메라 상향 기울기. 15cm 높이 수평 장착이면 근거리 사람 머리가
    // 수직 화각 밖이므로, 얼굴을 찍으려면 위로 기울여 설치해야 하고
    // 그 각도를 여기에 반영한다 (현장 실측 후 env 로 보정).
    double pitch_deg     = 15.0;
    bool   image_mirror  = false;  // 화면 좌우가 실물과 반대면 true
};

/**
 * @brief 라벨 기준점(라벨 하단 중앙) 픽셀 좌표 계산.
 * @param offset_deg   서보 조준각 대비 사람의 수평 각도 차 (도, +는 각도 증가 방향)
 * @param distance_mm  LiDAR 원점에서 사람까지 거리
 * @return 계산 가능하면 true. enable=false 또는 무효 입력이면 false.
 */
inline bool ComputeHeadLabelPos(const HeadLabelParams& p,
                                double offset_deg,
                                double distance_mm,
                                int img_w, int img_h,
                                int* out_x, int* out_y) {
    if (!p.enable || img_w <= 0 || img_h <= 0 || !out_x || !out_y) return false;
    if (p.hfov_deg <= 1.0 || p.vfov_deg <= 1.0) return false;

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kDeg2Rad = kPi / 180.0;

    double off = p.image_mirror ? -offset_deg : offset_deg;
    off = std::clamp(off, -80.0, 80.0);

    // 수평: 화각 절반의 tan 대비 비율로 정규화 (pin-hole 투영)
    const double half_w_tan = std::tan(p.hfov_deg * 0.5 * kDeg2Rad);
    const double x_norm = std::tan(off * kDeg2Rad) / half_w_tan;
    double px = (static_cast<double>(img_w) * 0.5) * (1.0 + x_norm);

    // 수직: 머리 고도각에서 카메라 피치를 뺀 상대 각도.
    //   거리가 가까울수록 라벨이 위로 올라간다.
    const double dist_m = std::max(distance_mm / 1000.0, 0.3);
    const double elev =
        std::atan2(p.head_height_m - p.cam_height_m, dist_m) -
        p.pitch_deg * kDeg2Rad;
    const double half_h_tan = std::tan(p.vfov_deg * 0.5 * kDeg2Rad);
    const double y_norm = std::tan(std::clamp(elev, -1.2, 1.2)) / half_h_tan;
    double py = (static_cast<double>(img_h) * 0.5) * (1.0 - y_norm);

    // 라벨이 사진 밖으로 나가지 않도록 클램프
    px = std::clamp(px, img_w * 0.05, img_w * 0.95);
    py = std::clamp(py, img_h * 0.04, img_h * 0.90);

    *out_x = static_cast<int>(std::lround(px));
    *out_y = static_cast<int>(std::lround(py));
    return true;
}

} // namespace ecowarden
