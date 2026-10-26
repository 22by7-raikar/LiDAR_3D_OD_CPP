#include "lidar_od/ground/ground_segmenter.h"
#include "plane_fit.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace lidar_od {

RansacGroundSegmenter::RansacGroundSegmenter(const RansacGroundParams& params,
                                             const SensorParams& sensor)
    : params_(params), sensor_(sensor) {}

void RansacGroundSegmenter::segment(const Cloud& cloud, const Indices& input, Indices& ground,
                                    Indices& nonground) {
    ground.clear();
    nonground.clear();
    plane_.setZero();
    iterations_used_ = 0;
    if (input.size() < 3) {
        nonground = input;
        return;
    }

    // Hypotheses are drawn only from low points: the road is below the sensor, so
    // this keeps building walls and car roofs from ever being proposed.
    Indices candidates;
    const float max_z = -sensor_.height + params_.sample_max_height;
    for (int i : input)
        if (cloud[i].z < max_z) candidates.push_back(i);
    if (candidates.size() < 3) candidates = input;

    const float min_normal_z = std::cos(params_.max_slope_deg * static_cast<float>(M_PI) / 180.0f);
    const float thr = params_.distance_threshold;
    const int n = static_cast<int>(input.size());

    std::mt19937 rng(params_.seed);
    std::uniform_int_distribution<size_t> pick(0, candidates.size() - 1);

    int best_count = 0;
    Eigen::Vector4f best = Eigen::Vector4f::Zero();
    int needed = params_.max_iterations;
    for (int it = 0; it < needed && it < params_.max_iterations; ++it) {
        ++iterations_used_;
        const size_t a = pick(rng), b = pick(rng), c = pick(rng);
        if (a == b || b == c || a == c) continue;
        const Eigen::Vector3f p1 = cloud[candidates[a]].getVector3fMap();
        Eigen::Vector3f normal = (cloud[candidates[b]].getVector3fMap() - p1)
                                     .cross(cloud[candidates[c]].getVector3fMap() - p1);
        const float len = normal.norm();
        if (len < 1e-6f) continue;  // collinear sample
        normal /= len;
        if (normal.z() < 0.0f) normal = -normal;
        if (normal.z() < min_normal_z) continue;  // too steep to be road
        const float d = -(normal.x() * p1.x() + normal.y() * p1.y() + normal.z() * p1.z());

        int count = 0;
#pragma omp parallel for reduction(+ : count) schedule(static)
        for (int k = 0; k < n; ++k) {
            const Point& p = cloud[input[k]];
            if (std::abs(normal.x() * p.x + normal.y() * p.y + normal.z() * p.z + d) <= thr) ++count;
        }

        if (count > best_count) {
            best_count = count;
            best = Eigen::Vector4f(normal.x(), normal.y(), normal.z(), d);
            // Adaptive stop: enough iterations to draw one all-inlier sample with
            // the requested probability, given the best inlier ratio so far.
            const double w = static_cast<double>(count) / n;
            const double denom = std::log(1.0 - w * w * w);
            if (denom < 0.0)
                needed = static_cast<int>(std::ceil(std::log(1.0 - params_.success_probability) / denom));
        }
    }

    if (best_count == 0) {
        nonground = input;
        return;
    }

    // Least-squares refinement on the consensus set removes the bias of the
    // three-point hypothesis.
    for (int r = 0; r < params_.refine_iterations; ++r) {
        Indices inliers;
        for (int i : input)
            if (std::abs(signedDistance(best, cloud[i])) <= thr) inliers.push_back(i);
        PlaneFit fit;
        if (!fitPlane(cloud, inliers, fit)) break;
        best = fit.plane;
    }
    plane_ = best;

    // One-sided test: points below the road (multipath, curbs at the low side)
    // are ground too, never obstacles.
    for (int i : input) (signedDistance(best, cloud[i]) <= thr ? ground : nonground).push_back(i);
}

}  // namespace lidar_od
