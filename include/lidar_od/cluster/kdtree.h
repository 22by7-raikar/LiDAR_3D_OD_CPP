/// @file
/// @brief Static 3D k-d tree for radius queries.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <vector>

namespace lidar_od {

/// @brief Static 3D k-d tree with radius search.
///
/// Built by median splits on the axis of largest spread, so the depth is
/// ⌈log2 n⌉ regardless of the input order. Nodes are stored implicitly: the
/// subtree over [lo, hi) splits at mid = (lo + hi) / 2.
class KdTree3 {
public:
    KdTree3() = default;

    /// @brief Builds the tree.
    /// @param points Points to index; ids returned by queries are positions in this vector.
    explicit KdTree3(std::vector<Eigen::Vector3f> points) { build(std::move(points)); }

    /// @brief Rebuilds the tree from new points.
    /// @param points Points to index.
    void build(std::vector<Eigen::Vector3f> points) {
        points_ = std::move(points);
        order_.resize(points_.size());
        for (size_t i = 0; i < order_.size(); ++i) order_[i] = static_cast<int>(i);
        axis_.assign(points_.size(), 0);
        buildRange(0, static_cast<int>(order_.size()));
    }

    /// @brief Number of indexed points.
    size_t size() const { return points_.size(); }

    /// @brief Indexed point by id.
    const Eigen::Vector3f& point(int id) const { return points_[id]; }

    /// @brief Finds all points within a radius.
    /// @param q Query point.
    /// @param radius Search radius.
    /// @param[out] out Ids of the points within @p radius are appended here.
    void radiusSearch(const Eigen::Vector3f& q, float radius, std::vector<int>& out) const {
        const float r2 = radius * radius;
        struct Item { int lo, hi; };
        Item stack[128];  // depth <= log2(n) + 1, so this never overflows
        int top = 0;
        if (!order_.empty()) stack[top++] = {0, static_cast<int>(order_.size())};
        while (top > 0) {
            const Item it = stack[--top];
            if (it.lo >= it.hi) continue;
            const int mid = (it.lo + it.hi) / 2;
            const int id = order_[mid];
            const Eigen::Vector3f& p = points_[id];
            if ((p - q).squaredNorm() <= r2) out.push_back(id);
            const int ax = axis_[mid];
            const float diff = q[ax] - p[ax];
            // Visit the near side always and the far side only if the ball crosses the split.
            if (diff <= radius) stack[top++] = {it.lo, mid};
            if (diff >= -radius) stack[top++] = {mid + 1, it.hi};
        }
    }

private:
    // Recursion depth is bounded by log2(n) thanks to the median splits.
    void buildRange(int lo, int hi) {
        if (hi - lo <= 0) return;
        // Split on the axis with the largest spread: driving scenes are much
        // wider than they are tall, so this beats cycling through x, y, z.
        Eigen::Vector3f lo_b = points_[order_[lo]], hi_b = lo_b;
        for (int i = lo + 1; i < hi; ++i) {
            lo_b = lo_b.cwiseMin(points_[order_[i]]);
            hi_b = hi_b.cwiseMax(points_[order_[i]]);
        }
        int ax;
        (hi_b - lo_b).maxCoeff(&ax);
        const int mid = (lo + hi) / 2;
        std::nth_element(order_.begin() + lo, order_.begin() + mid, order_.begin() + hi,
                         [&](int a, int b) { return points_[a][ax] < points_[b][ax]; });
        axis_[mid] = static_cast<unsigned char>(ax);
        buildRange(lo, mid);
        buildRange(mid + 1, hi);
    }

    std::vector<Eigen::Vector3f> points_;
    std::vector<int> order_;
    std::vector<unsigned char> axis_;
};

}  // namespace lidar_od
