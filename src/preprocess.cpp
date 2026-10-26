#include "lidar_od/preprocess.h"

#include <cmath>
#include <limits>
#include <unordered_map>

namespace lidar_od {
namespace {

bool inside(const Point& p, const Eigen::Vector3f& lo, const Eigen::Vector3f& hi) {
    return p.x >= lo.x() && p.x <= hi.x() && p.y >= lo.y() && p.y <= hi.y() &&
           p.z >= lo.z() && p.z <= hi.z();
}

}  // namespace

Indices cropToRoi(const Cloud& cloud, const PreprocessParams& params) {
    Indices out;
    out.reserve(cloud.size());
    for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
        const Point& p = cloud[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (!inside(p, params.roi_min, params.roi_max)) continue;
        if (params.remove_ego && inside(p, params.ego_min, params.ego_max)) continue;
        out.push_back(i);
    }
    return out;
}

Indices voxelDownsample(const Cloud& cloud, const Indices& input, float leaf) {
    if (leaf <= 0.0f) return input;

    struct Voxel {
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        int count = 0;
        int best = -1;
        float best_d2 = std::numeric_limits<float>::max();
    };
    auto key = [leaf](const Point& p) {
        const auto ix = static_cast<int64_t>(std::floor(p.x / leaf)) & 0x1FFFFF;
        const auto iy = static_cast<int64_t>(std::floor(p.y / leaf)) & 0x1FFFFF;
        const auto iz = static_cast<int64_t>(std::floor(p.z / leaf)) & 0x1FFFFF;
        return (ix << 42) | (iy << 21) | iz;
    };

    std::unordered_map<int64_t, Voxel> voxels;
    voxels.reserve(input.size());
    for (int i : input) {
        Voxel& v = voxels[key(cloud[i])];
        v.sum += cloud[i].getVector3fMap();
        ++v.count;
    }
    for (int i : input) {
        Voxel& v = voxels[key(cloud[i])];
        const float d2 = (cloud[i].getVector3fMap() - v.sum / static_cast<float>(v.count)).squaredNorm();
        if (d2 < v.best_d2) {
            v.best_d2 = d2;
            v.best = i;
        }
    }

    // Emit in input order to keep the output deterministic.
    Indices out;
    out.reserve(voxels.size());
    for (int i : input)
        if (voxels[key(cloud[i])].best == i) out.push_back(i);
    return out;
}

}  // namespace lidar_od
