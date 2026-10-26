#include "lidar_od/ground/ground_segmenter.h"

#include <algorithm>
#include <cmath>

namespace lidar_od {

RayGroundSegmenter::RayGroundSegmenter(const RayGroundParams& params, const SensorParams& sensor)
    : params_(params), sensor_(sensor) {}

void RayGroundSegmenter::segment(const Cloud& cloud, const Indices& input, Indices& ground,
                                 Indices& nonground) {
    ground.clear();
    nonground.clear();

    const int sectors = std::max(1, static_cast<int>(std::ceil(360.0f / params_.sector_deg)));
    struct RayPoint {
        float range;
        float z;
        int index;
    };
    std::vector<std::vector<RayPoint>> rays(sectors);
    for (int i : input) {
        const Point& p = cloud[i];
        float az = std::atan2(p.y, p.x) * 180.0f / static_cast<float>(M_PI);
        if (az < 0.0f) az += 360.0f;
        const int s = std::min(static_cast<int>(az / params_.sector_deg), sectors - 1);
        rays[s].push_back({std::hypot(p.x, p.y), p.z, i});
    }

    const float tan_local = std::tan(params_.local_max_slope_deg * static_cast<float>(M_PI) / 180.0f);
    const float tan_general = std::tan(params_.general_max_slope_deg * static_cast<float>(M_PI) / 180.0f);
    const float road = -sensor_.height;

    for (auto& ray : rays) {
        std::sort(ray.begin(), ray.end(), [](const RayPoint& a, const RayPoint& b) { return a.range < b.range; });

        // Walk outwards from the sensor footprint, following the last ground point.
        float ground_r = 0.0f;
        float ground_z = road;
        for (const RayPoint& p : ray) {
            if (p.z < road - params_.noise_floor) {  // reflection below the road
                ground.push_back(p.index);
                continue;
            }
            const float dr = p.range - ground_r;
            const float general_thr = std::max(tan_general * p.range, params_.min_height_threshold);
            const bool near_road = std::abs(p.z - road) <= general_thr;

            bool is_ground;
            if (dr > params_.reclass_distance) {
                // Long gap (occlusion): the last ground point says little, so
                // re-anchor on the global ground model.
                is_ground = near_road;
            } else {
                const float local_thr = std::max(tan_local * dr, params_.min_height_threshold);
                is_ground = std::abs(p.z - ground_z) <= local_thr && near_road;
            }

            if (is_ground) {
                ground_r = p.range;
                ground_z = p.z;
                ground.push_back(p.index);
            } else {
                nonground.push_back(p.index);
            }
        }
    }
}

}  // namespace lidar_od
