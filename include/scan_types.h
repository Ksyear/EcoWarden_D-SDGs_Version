/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace ecowarden {

// LiDAR scan data shared by hardware drivers and offline simulation.
struct ScanPoint {
    float    angle_deg;   // angle in degrees
    uint16_t distance_mm; // distance in millimeters
    uint8_t  intensity;   // return intensity
};

using ScanFrame = std::vector<ScanPoint>;

} // namespace ecowarden
