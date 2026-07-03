#pragma once

#include <algorithm>
#include <cmath>

namespace ecowarden {

inline double ZoneCenterDeg(int zone_index) {
    static constexpr double kCenters[5] = {0.0, 45.0, 90.0, 135.0, 180.0};
    zone_index = std::clamp(zone_index, 0, 4);
    return kCenters[zone_index];
}

// LiDAR 좌표 → 서보 조준각(0~180°). mirror 보정까지 포함한 값을 돌려준다.
inline double SuspectAngleDeg(double x_mm, double y_mm, bool servo_mirror) {
    static constexpr double kPi = 3.14159265358979323846;
    const double forward_mm = std::max(std::abs(y_mm), 1.0);
    double angle = std::atan2(x_mm, forward_mm) * 180.0 / kPi + 90.0;
    angle = std::clamp(angle, 0.0, 180.0);
    if (servo_mirror) angle = 180.0 - angle;
    return angle;
}

inline int SuspectToZone(double x_mm, double y_mm, bool servo_mirror) {
    const double angle = SuspectAngleDeg(x_mm, y_mm, servo_mirror);
    const int zone = static_cast<int>(std::round(angle / 45.0));
    return std::clamp(zone, 0, 4);
}

} // namespace ecowarden
