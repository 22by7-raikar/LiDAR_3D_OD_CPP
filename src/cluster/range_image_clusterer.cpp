#include "lidar_od/cluster/clusterer.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>
#include <unordered_map>

namespace lidar_od {

namespace {

// Surfaces seen at grazing incidence (long walls, the side of a bus alongside
// us) have large depth jumps between neighbouring pixels, so the angle
// criterion cuts them into pieces even though the pieces touch in 3D. Joining
// clusters that occupy a common small voxel restores them at O(n) cost.
std::vector<Indices> mergeTouching(const Cloud& cloud, std::vector<Indices> clusters, float voxel) {
    std::vector<int> parent(clusters.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](int x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };

    std::unordered_map<int64_t, int> owner;
    for (size_t c = 0; c < clusters.size(); ++c)
        for (int i : clusters[c]) {
            const Point& pt = cloud[i];
            const int64_t key = ((static_cast<int64_t>(std::floor(pt.x / voxel)) & 0x1FFFFF) << 42) |
                                ((static_cast<int64_t>(std::floor(pt.y / voxel)) & 0x1FFFFF) << 21) |
                                (static_cast<int64_t>(std::floor(pt.z / voxel)) & 0x1FFFFF);
            auto [it, inserted] = owner.try_emplace(key, static_cast<int>(c));
            if (!inserted) {
                const int a = find(it->second), b = find(static_cast<int>(c));
                if (a != b) parent[std::max(a, b)] = std::min(a, b);
            }
        }

    std::vector<Indices> merged(clusters.size());
    for (size_t c = 0; c < clusters.size(); ++c) {
        auto& dst = merged[find(static_cast<int>(c))];
        dst.insert(dst.end(), clusters[c].begin(), clusters[c].end());
    }
    merged.erase(std::remove_if(merged.begin(), merged.end(), [](const Indices& c) { return c.empty(); }),
                 merged.end());
    return merged;
}

}  // namespace

RangeImageClusterer::RangeImageClusterer(const ClusterParams& params, const SensorParams& sensor)
    : params_(params), sensor_(sensor) {}

std::vector<Indices> RangeImageClusterer::cluster(const Cloud& cloud, const ScanLayout* layout,
                                                  const Indices& input) {
    const auto& p = params_.range_image;
    ScanLayout own;
    if (layout == nullptr || layout->row.size() != cloud.size()) {
        own = computeScanLayout(cloud, sensor_, p.use_ring_order);
        layout = &own;
    }
    const int rows = layout->rows, cols = layout->cols;

    // Project the non-ground points; on collisions keep the closest return.
    std::vector<int> image(static_cast<size_t>(rows) * cols, -1);
    std::vector<float> range(cloud.size(), 0.0f);
    std::vector<int> collided;
    for (int i : input) {
        const int r = layout->row[i], c = layout->col[i];
        if (r < 0) continue;
        range[i] = cloud[i].getVector3fMap().norm();
        int& px = image[static_cast<size_t>(r) * cols + c];
        if (px < 0) {
            px = i;
        } else if (range[i] < range[px]) {
            collided.push_back(px);
            px = i;
        } else {
            collided.push_back(i);
        }
    }

    const float tan_thr = std::tan(p.angle_threshold_deg * static_cast<float>(M_PI) / 180.0f);
    const float max_d2 = p.max_distance * p.max_distance;

    // Two neighbouring returns at ranges d1 >= d2 separated by the beam angle
    // alpha belong together when beta = atan2(d2 sin(alpha), d1 - d2 cos(alpha))
    // exceeds the threshold. beta stays large along a continuous surface and
    // collapses at the depth jump between two objects.
    auto connected = [&](int a, int b) {
        const Eigen::Vector3f pa = cloud[a].getVector3fMap(), pb = cloud[b].getVector3fMap();
        if ((pa - pb).squaredNorm() > max_d2) return false;
        const float d1 = std::max(range[a], range[b]);
        const float d2 = std::min(range[a], range[b]);
        const float cos_a = std::clamp(pa.dot(pb) / (range[a] * range[b]), -1.0f, 1.0f);
        const float sin_a = std::sqrt(1.0f - cos_a * cos_a);
        // beta > thr  <=>  d2 sin(a) > tan(thr) * (d1 - d2 cos(a)), avoiding atan2.
        return d2 * sin_a > tan_thr * (d1 - d2 * cos_a);
    };

    std::vector<int> label(image.size(), -1);
    std::vector<Indices> clusters;
    std::deque<int> queue;
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    for (size_t start = 0; start < image.size(); ++start) {
        if (image[start] < 0 || label[start] >= 0) continue;
        const int id = static_cast<int>(clusters.size());
        clusters.emplace_back();
        label[start] = id;
        queue.assign(1, static_cast<int>(start));
        while (!queue.empty()) {
            const int cur = queue.front();
            queue.pop_front();
            clusters[id].push_back(image[cur]);
            const int r0 = cur / cols, c0 = cur % cols;
            for (int d = 0; d < 4; ++d) {
                // Step over a few empty pixels (dropped returns, removed ground)
                // and connect to the first occupied one.
                const int max_step = 1 + (dr[d] != 0 ? p.max_row_gap : p.max_col_gap);
                for (int s = 1; s <= max_step; ++s) {
                    const int r = r0 + dr[d] * s;
                    if (r < 0 || r >= rows) break;
                    const int c = ((c0 + dc[d] * s) % cols + cols) % cols;  // azimuth wraps around
                    const int nb = r * cols + c;
                    if (image[nb] < 0) continue;
                    if (label[nb] < 0 && connected(image[cur], image[nb])) {
                        label[nb] = id;
                        queue.push_back(nb);
                    }
                    break;
                }
            }
        }
    }

    // Points hidden behind a closer return in the same pixel join that pixel's
    // object when they are close to it.
    for (int i : collided) {
        const size_t px = static_cast<size_t>(layout->row[i]) * cols + layout->col[i];
        const int owner = image[px];
        if (owner >= 0 && label[px] >= 0 &&
            (cloud[i].getVector3fMap() - cloud[owner].getVector3fMap()).squaredNorm() <= max_d2)
            clusters[label[px]].push_back(i);
    }

    if (p.merge_voxel > 0.0f) clusters = mergeTouching(cloud, std::move(clusters), p.merge_voxel);

    std::vector<Indices> out;
    for (auto& c : clusters)
        if (static_cast<int>(c.size()) >= params_.min_points &&
            static_cast<int>(c.size()) <= params_.max_points)
            out.push_back(std::move(c));
    return out;
}

}  // namespace lidar_od
