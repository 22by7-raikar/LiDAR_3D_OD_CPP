#include "lidar_od/box/box_fitter.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lidar_od {

std::unique_ptr<BoxFitter> makeBoxFitter(const BoxParams& params) {
    if (params.method == "lshape")
        return std::make_unique<LShapeFitter>(LShapeFitter::parseCriterion(params.criterion),
                                              params.angle_step_deg, params.ground_snap);
    if (params.method == "pca") return std::make_unique<PcaFitter>(params.ground_snap);
    if (params.method == "aabb") return std::make_unique<AabbFitter>(params.ground_snap);
    throw std::invalid_argument("unknown box method '" + params.method + "'");
}

OrientedBox BoxFitter::fit(const Cloud& cloud, const Indices& cluster, float ground_z) const {
    std::vector<Eigen::Vector2f> xy;
    xy.reserve(cluster.size());
    float zmin = std::numeric_limits<float>::max(), zmax = std::numeric_limits<float>::lowest();
    for (int i : cluster) {
        xy.emplace_back(cloud[i].x, cloud[i].y);
        zmin = std::min(zmin, cloud[i].z);
        zmax = std::max(zmax, cloud[i].z);
    }

    // The heading search is O(points x angles); a few hundred evenly spread
    // points describe the edges just as well as thousands.
    constexpr size_t kMaxHeadingPoints = 256;
    float theta = 0.0f;
    if (xy.size() > kMaxHeadingPoints) {
        std::vector<Eigen::Vector2f> sample;
        sample.reserve(kMaxHeadingPoints);
        const double stride = static_cast<double>(xy.size()) / kMaxHeadingPoints;
        for (size_t k = 0; k < kMaxHeadingPoints; ++k) sample.push_back(xy[static_cast<size_t>(k * stride)]);
        theta = heading(sample);
    } else if (xy.size() >= 3) {
        theta = heading(xy);
    }
    const Eigen::Vector2f e1(std::cos(theta), std::sin(theta));
    const Eigen::Vector2f e2(-e1.y(), e1.x());
    float a_min = std::numeric_limits<float>::max(), a_max = std::numeric_limits<float>::lowest();
    float b_min = a_min, b_max = a_max;
    for (const auto& p : xy) {
        const float a = p.dot(e1), b = p.dot(e2);
        a_min = std::min(a_min, a);
        a_max = std::max(a_max, a);
        b_min = std::min(b_min, b);
        b_max = std::max(b_max, b);
    }

    // The lowest returns of an object are usually a few decimetres above the
    // road (bumpers, occlusion); put the box on the ground when it is close.
    if (zmin - ground_z >= 0.0f && zmin - ground_z <= ground_snap_) zmin = ground_z;

    OrientedBox box;
    const Eigen::Vector2f c = 0.5f * (a_min + a_max) * e1 + 0.5f * (b_min + b_max) * e2;
    box.center << c, 0.5f * (zmin + zmax);
    float length = a_max - a_min, width = b_max - b_min, yaw = theta;
    if (width > length) {  // length is always the longer side
        std::swap(length, width);
        yaw += static_cast<float>(M_PI_2);
    }
    box.size << length, width, zmax - zmin;
    box.yaw = wrapHalfPi(yaw);
    return box;
}

float PcaFitter::heading(const std::vector<Eigen::Vector2f>& xy) const {
    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    for (const auto& p : xy) mean += p;
    mean /= static_cast<float>(xy.size());
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : xy) cov += (p - mean) * (p - mean).transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
    const Eigen::Vector2f major = es.eigenvectors().col(1);
    float t = std::atan2(major.y(), major.x());
    t = std::fmod(t + 2.0f * static_cast<float>(M_PI), static_cast<float>(M_PI_2));
    return t;
}

LShapeFitter::LShapeFitter(Criterion criterion, float angle_step_deg, float ground_snap)
    : BoxFitter(ground_snap), criterion_(criterion), step_(angle_step_deg * static_cast<float>(M_PI) / 180.0f) {
    if (step_ <= 0.0f) throw std::invalid_argument("lshape: angle_step_deg must be positive");
}

LShapeFitter::Criterion LShapeFitter::parseCriterion(const std::string& s) {
    if (s == "area") return Criterion::Area;
    if (s == "closeness") return Criterion::Closeness;
    if (s == "variance") return Criterion::Variance;
    throw std::invalid_argument("unknown lshape criterion '" + s + "'");
}

float LShapeFitter::score(const std::vector<Eigen::Vector2f>& xy, float theta) const {
    const Eigen::Vector2f e1(std::cos(theta), std::sin(theta));
    const Eigen::Vector2f e2(-e1.y(), e1.x());
    const size_t n = xy.size();
    std::vector<float> c1(n), c2(n);
    for (size_t i = 0; i < n; ++i) {
        c1[i] = xy[i].dot(e1);
        c2[i] = xy[i].dot(e2);
    }
    const auto [min1, max1] = std::minmax_element(c1.begin(), c1.end());
    const auto [min2, max2] = std::minmax_element(c2.begin(), c2.end());

    switch (criterion_) {
        case Criterion::Area:
            return -(*max1 - *min1) * (*max2 - *min2);

        case Criterion::Closeness: {
            // Reward points lying close to the nearer of the two closest edges.
            constexpr float d0 = 0.01f;
            float s = 0.0f;
            for (size_t i = 0; i < n; ++i) {
                const float d1 = std::min(*max1 - c1[i], c1[i] - *min1);
                const float d2 = std::min(*max2 - c2[i], c2[i] - *min2);
                s += 1.0f / std::max(std::min(d1, d2), d0);
            }
            return s;
        }

        case Criterion::Variance: {
            // Assign each point to its nearer edge, then penalise the spread of
            // distances to each edge: low variance means two crisp sides.
            std::vector<float> e1d, e2d;
            for (size_t i = 0; i < n; ++i) {
                const float d1 = std::min(*max1 - c1[i], c1[i] - *min1);
                const float d2 = std::min(*max2 - c2[i], c2[i] - *min2);
                (d1 < d2 ? e1d : e2d).push_back(d1 < d2 ? d1 : d2);
            }
            auto variance = [](const std::vector<float>& v) {
                if (v.empty()) return 0.0f;
                float m = 0.0f;
                for (float x : v) m += x;
                m /= static_cast<float>(v.size());
                float s = 0.0f;
                for (float x : v) s += (x - m) * (x - m);
                return s / static_cast<float>(v.size());
            };
            return -(variance(e1d) + variance(e2d));
        }
    }
    return 0.0f;
}

float LShapeFitter::heading(const std::vector<Eigen::Vector2f>& xy) const {
    float best = 0.0f, best_score = std::numeric_limits<float>::lowest();
    for (float t = 0.0f; t < static_cast<float>(M_PI_2); t += step_) {
        const float s = score(xy, t);
        if (s > best_score) {
            best_score = s;
            best = t;
        }
    }
    return best;
}

}  // namespace lidar_od
