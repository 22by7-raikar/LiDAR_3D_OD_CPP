#include "lidar_od/tracking/hungarian.h"
#include "lidar_od/tracking/kalman_filter.h"
#include "lidar_od/tracking/tracker.h"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>
#include <random>

using namespace lidar_od;

namespace {

double bruteForce(const std::vector<std::vector<double>>& cost) {
    const size_t rows = cost.size(), cols = cost[0].size();
    std::vector<int> perm(std::max(rows, cols));
    std::iota(perm.begin(), perm.end(), 0);
    double best = 1e18;
    do {
        double s = 0.0;
        for (size_t r = 0; r < rows; ++r)
            if (perm[r] < static_cast<int>(cols)) s += cost[r][perm[r]];
            else if (rows <= cols) s += 1e9;  // a row must be assigned when possible
        best = std::min(best, s);
    } while (std::next_permutation(perm.begin(), perm.end()));
    return best;
}

OrientedBox boxAt(float x, float y) {
    OrientedBox b;
    b.center = Eigen::Vector3f(x, y, -1.0f);
    b.size = Eigen::Vector3f(4.0f, 1.8f, 1.5f);
    return b;
}

}  // namespace

TEST(Hungarian, MatchesBruteForceOnRandomMatrices) {
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(0.0, 10.0);
    for (const auto& [rows, cols] : std::vector<std::pair<int, int>>{{4, 4}, {3, 6}, {6, 3}, {5, 5}, {1, 4}}) {
        for (int trial = 0; trial < 20; ++trial) {
            std::vector<std::vector<double>> cost(rows, std::vector<double>(cols));
            for (auto& row : cost)
                for (double& c : row) c = u(rng);
            const auto a = solveAssignment(cost);
            double total = 0.0;
            std::vector<int> used;
            for (int r = 0; r < rows; ++r)
                if (a[r] >= 0) {
                    total += cost[r][a[r]];
                    used.push_back(a[r]);
                }
            std::sort(used.begin(), used.end());
            EXPECT_EQ(std::adjacent_find(used.begin(), used.end()), used.end());
            EXPECT_EQ(static_cast<int>(used.size()), std::min(rows, cols));
            EXPECT_NEAR(total, bruteForce(cost), 1e-9);
        }
    }
}

TEST(Kalman, ConvergesToConstantVelocity) {
    KalmanParams params;
    std::mt19937 rng(2);
    std::normal_distribution<float> noise(0.0f, params.measurement_noise);
    const Eigen::Vector2f v(6.0f, -1.5f);
    ConstantVelocityKalman kf(params, Eigen::Vector2f(0.0f, 0.0f));
    for (int k = 1; k <= 60; ++k) {
        kf.predict(0.1f);
        const Eigen::Vector2f truth = v * (0.1f * k);
        kf.update(truth + Eigen::Vector2f(noise(rng), noise(rng)));
    }
    // The estimate must agree with the truth within the filter's own 3-sigma bound.
    const Eigen::Matrix4f& Pv = kf.covariance();
    EXPECT_NEAR(kf.velocity().x(), v.x(), 3.0f * std::sqrt(Pv(2, 2)));
    EXPECT_NEAR(kf.velocity().y(), v.y(), 3.0f * std::sqrt(Pv(3, 3)));
    EXPECT_LT(std::sqrt(Pv(2, 2)), 1.0f);  // and that bound must be informative
    // Covariance must stay symmetric positive definite.
    const Eigen::Matrix4f P = kf.covariance();
    EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-5f);
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f>(P).eigenvalues().minCoeff(), 0.0f);
}

TEST(Tracker, KeepsIdentitiesThroughCrossingAndShortOcclusion) {
    TrackerParams params;
    MultiObjectTracker tracker(params);
    // Two cars on parallel lanes 3.5 m apart, driving in opposite directions,
    // passing each other; car B is missed for two frames mid-way.
    std::map<int, int> id_of;  // car -> track id
    for (int k = 0; k < 40; ++k) {
        const float t = 0.1f * static_cast<float>(k);
        std::vector<OrientedBox> det{boxAt(-10.0f + 8.0f * t, 0.0f)};
        const bool b_visible = k < 18 || k > 19;
        if (b_visible) det.push_back(boxAt(10.0f - 8.0f * t, 3.5f));
        tracker.update(det, t);

        for (const Track& tr : tracker.tracks()) {
            if (tr.state != TrackState::Confirmed || tr.detection < 0) continue;
            const int car = tr.detection;  // detections are listed A then B
            if (!id_of.count(car)) id_of[car] = tr.id;
            EXPECT_EQ(id_of[car], tr.id) << "identity switch at frame " << k;
        }
    }
    ASSERT_EQ(id_of.size(), 2u);
    for (const Track& tr : tracker.tracks()) {
        EXPECT_EQ(tr.state, TrackState::Confirmed);
        EXPECT_NEAR(std::abs(tr.kf.velocity().x()), 8.0f, 0.5f);
    }
}

TEST(Tracker, DropsClutterThatIsNotConfirmed) {
    TrackerParams params;
    MultiObjectTracker tracker(params);
    tracker.update({boxAt(5.0f, 5.0f)}, 0.0);
    EXPECT_EQ(tracker.tracks().size(), 1u);
    tracker.update({}, 0.1);
    tracker.update({}, 0.2);
    EXPECT_TRUE(tracker.tracks().empty());
}
