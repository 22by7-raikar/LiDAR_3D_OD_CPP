#include "lidar_od/cluster/clusterer.h"
#include "lidar_od/cluster/kdtree.h"
#include "lidar_od/preprocess.h"

#include <cmath>
#include <deque>

namespace lidar_od {

EuclideanClusterer::EuclideanClusterer(const ClusterParams& params) : params_(params) {}

std::vector<Indices> EuclideanClusterer::cluster(const Cloud& cloud, const ScanLayout*,
                                                 const Indices& input) {
    const auto& p = params_.euclidean;
    const Indices reps = voxelDownsample(cloud, input, p.voxel_leaf);

    std::vector<Eigen::Vector3f> pts;
    pts.reserve(reps.size());
    for (int i : reps) pts.push_back(cloud[i].getVector3fMap());
    const KdTree3 tree(pts);

    // Breadth-first region growing with an explicit queue: no recursion, so no
    // stack overflow on large clusters such as walls.
    std::vector<int> label(reps.size(), -1);
    std::vector<int> neighbours;
    int next = 0;
    for (size_t seed = 0; seed < reps.size(); ++seed) {
        if (label[seed] >= 0) continue;
        label[seed] = next;
        std::deque<int> queue{static_cast<int>(seed)};
        while (!queue.empty()) {
            const int cur = queue.front();
            queue.pop_front();
            const float range = pts[cur].head<2>().norm();
            neighbours.clear();
            tree.radiusSearch(pts[cur], p.tolerance + p.tolerance_per_meter * range, neighbours);
            for (int n : neighbours)
                if (label[n] < 0) {
                    label[n] = next;
                    queue.push_back(n);
                }
        }
        ++next;
    }

    // Give every original point the label of its voxel representative.
    std::vector<Indices> clusters(next);
    if (reps.size() == input.size()) {
        for (size_t k = 0; k < reps.size(); ++k) clusters[label[k]].push_back(reps[k]);
    } else {
        const float r = p.voxel_leaf * 1.75f;  // covers the voxel diagonal
        for (int i : input) {
            const Eigen::Vector3f q = cloud[i].getVector3fMap();
            neighbours.clear();
            tree.radiusSearch(q, r, neighbours);
            int best = -1;
            float best_d2 = r * r;
            for (int n : neighbours) {
                const float d2 = (pts[n] - q).squaredNorm();
                if (d2 <= best_d2) {
                    best_d2 = d2;
                    best = n;
                }
            }
            if (best >= 0) clusters[label[best]].push_back(i);
        }
    }

    std::vector<Indices> out;
    for (auto& c : clusters)
        if (static_cast<int>(c.size()) >= params_.min_points &&
            static_cast<int>(c.size()) <= params_.max_points)
            out.push_back(std::move(c));
    return out;
}

}  // namespace lidar_od
