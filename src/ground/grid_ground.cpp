#include "lidar_od/ground/ground_segmenter.h"

#include <cmath>
#include <limits>
#include <unordered_map>

namespace lidar_od {

GridGroundSegmenter::GridGroundSegmenter(const GridGroundParams& params, const SensorParams& sensor)
    : params_(params), sensor_(sensor) {}

void GridGroundSegmenter::segment(const Cloud& cloud, const Indices& input, Indices& ground,
                                  Indices& nonground) {
    ground.clear();
    nonground.clear();

    auto key = [](int ix, int iy) { return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy); };
    auto cellOf = [this](float v) { return static_cast<int>(std::floor(v / params_.cell_size)); };

    // Returns well below the road (reflections) would drag the cell minimum
    // down and turn the real road above them into "obstacles".
    const float noise_floor = -sensor_.height - params_.noise_floor;
    std::unordered_map<int64_t, float> min_z;
    min_z.reserve(input.size() / 4);
    for (int i : input) {
        const Point& p = cloud[i];
        if (p.z < noise_floor) continue;
        auto [it, inserted] = min_z.try_emplace(key(cellOf(p.x), cellOf(p.y)), p.z);
        if (!inserted) it->second = std::min(it->second, p.z);
    }

    // The lowest point of the neighbourhood is the ground estimate: a cell fully
    // covered by a car roof still sees the road in a neighbouring cell.
    std::unordered_map<int64_t, float> local_ground;
    local_ground.reserve(min_z.size());
    const int n = params_.neighbourhood;
    const float max_ground = -sensor_.height + params_.max_ground_height;
    for (int i : input) {
        const Point& p = cloud[i];
        const int cx = cellOf(p.x), cy = cellOf(p.y);
        auto [it, inserted] = local_ground.try_emplace(key(cx, cy), 0.0f);
        if (inserted) {
            float m = std::numeric_limits<float>::max();
            for (int dx = -n; dx <= n; ++dx)
                for (int dy = -n; dy <= n; ++dy) {
                    auto c = min_z.find(key(cx + dx, cy + dy));
                    if (c != min_z.end()) m = std::min(m, c->second);
                }
            it->second = m;
        }
        const float g = it->second;
        const bool is_ground =
            p.z < noise_floor || (g < max_ground && p.z < g + params_.height_threshold);
        (is_ground ? ground : nonground).push_back(i);
    }
}

}  // namespace lidar_od
