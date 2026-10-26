#include "lidar_od/ground/ground_segmenter.h"

#include <cmath>
#include <stdexcept>

namespace lidar_od {

std::unique_ptr<GroundSegmenter> makeGroundSegmenter(const std::string& method,
                                                     const GroundParams& params,
                                                     const SensorParams& sensor) {
    if (method == "ransac") return std::make_unique<RansacGroundSegmenter>(params.ransac, sensor);
    if (method == "region") return std::make_unique<RegionGroundSegmenter>(params.region, sensor);
    if (method == "ray") return std::make_unique<RayGroundSegmenter>(params.ray, sensor);
    if (method == "grid") return std::make_unique<GridGroundSegmenter>(params.grid, sensor);
    throw std::invalid_argument("unknown ground method '" + method + "'");
}

GroundHeightMap::GroundHeightMap(float cell_size, float default_height)
    : cell_(cell_size), default_(default_height) {}

void GroundHeightMap::build(const Cloud& cloud, const Indices& ground) {
    cells_.clear();
    for (int i : ground) {
        const Point& p = cloud[i];
        auto& c = cells_[key(static_cast<int>(std::floor(p.x / cell_)),
                             static_cast<int>(std::floor(p.y / cell_)))];
        c.first += p.z;
        ++c.second;
    }
}

float GroundHeightMap::heightAt(float x, float y) const {
    const int cx = static_cast<int>(std::floor(x / cell_));
    const int cy = static_cast<int>(std::floor(y / cell_));
    // Objects hide the ground right under them, so search growing rings of cells.
    for (int r = 0; r <= 3; ++r) {
        float sum = 0.0f;
        int n = 0;
        for (int dx = -r; dx <= r; ++dx)
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
                auto it = cells_.find(key(cx + dx, cy + dy));
                if (it == cells_.end()) continue;
                sum += it->second.first;
                n += it->second.second;
            }
        if (n > 0) return sum / static_cast<float>(n);
    }
    return default_;
}

}  // namespace lidar_od
