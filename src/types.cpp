#include "lidar_od/types.h"

#include <cmath>

namespace lidar_od {

std::vector<Eigen::Vector2f> OrientedBox::footprint() const {
    const Eigen::Vector2f c = center.head<2>();
    const Eigen::Vector2f ax(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2f ay(-ax.y(), ax.x());
    const Eigen::Vector2f hl = 0.5f * size.x() * ax;
    const Eigen::Vector2f hw = 0.5f * size.y() * ay;
    return {c + hl + hw, c - hl + hw, c - hl - hw, c + hl - hw};
}

float wrapAngle(float a) {
    a = std::fmod(a + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (a <= 0.0f) a += 2.0f * static_cast<float>(M_PI);
    return a - static_cast<float>(M_PI);
}

float wrapHalfPi(float a) {
    a = std::fmod(a + static_cast<float>(M_PI_2), static_cast<float>(M_PI));
    if (a <= 0.0f) a += static_cast<float>(M_PI);
    return a - static_cast<float>(M_PI_2);
}

}  // namespace lidar_od
