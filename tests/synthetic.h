#pragma once

// Small synthetic scenes with known ground truth, shared by the tests.

#include "lidar_od/types.h"

#include <cmath>
#include <random>

namespace lidar_od::test {

inline Point makePoint(float x, float y, float z) {
    Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.0f;
    return p;
}

// Ray-casts a simple 64-ring scanner (stored ring by ring, like KITTI raw) at
// height `h` over flat ground with axis-aligned boxes. `is_object` marks the
// returns that hit a box.
struct Scene {
    struct Box {
        float x0, x1, y0, y1, z1;  // footprint and top; bottom is the ground
    };
    std::vector<Box> boxes;
    float sensor_height = 1.73f;
    float max_range = 40.0f;
    int rings = 64;
    int columns = 900;

    Cloud scan(std::vector<int>& is_object, unsigned seed = 1) const {
        std::mt19937 rng(seed);
        std::normal_distribution<float> noise(0.0f, 0.01f);
        Cloud cloud;
        is_object.clear();
        for (int r = 0; r < rings; ++r) {
            const float el = (2.0f - (26.9f * r) / (rings - 1)) * static_cast<float>(M_PI) / 180.0f;
            for (int c = 0; c < columns; ++c) {
                const float az = 2.0f * static_cast<float>(M_PI) * c / columns;
                const Eigen::Vector3f dir(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
                const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
                float best = max_range;
                bool object = false;
                if (dir.z() < 0.0f) best = std::min(best, -sensor_height / dir.z());
                for (const Box& b : boxes) {
                    // Slab intersection with [x0,x1]x[y0,y1]x[-h,z1].
                    float t0 = 0.0f, t1 = max_range;
                    const float lo[3] = {b.x0, b.y0, -sensor_height}, hi[3] = {b.x1, b.y1, b.z1};
                    bool hit = true;
                    for (int k = 0; k < 3 && hit; ++k) {
                        if (std::abs(dir[k]) < 1e-9f) {
                            hit = origin[k] >= lo[k] && origin[k] <= hi[k];
                            continue;
                        }
                        float a = (lo[k] - origin[k]) / dir[k], bb = (hi[k] - origin[k]) / dir[k];
                        if (a > bb) std::swap(a, bb);
                        t0 = std::max(t0, a);
                        t1 = std::min(t1, bb);
                        hit = t0 <= t1;
                    }
                    if (hit && t0 > 0.0f && t0 < best - 1e-4f) {
                        best = t0;
                        object = true;
                    }
                }
                if (best >= max_range) continue;
                const Eigen::Vector3f p = dir * (best + noise(rng));
                cloud.push_back(makePoint(p.x(), p.y(), p.z()));
                is_object.push_back(object ? 1 : 0);
            }
        }
        return cloud;
    }
};

}  // namespace lidar_od::test
