#include "lidar_od/box/box_fitter.h"
#include "synthetic.h"

#include <gtest/gtest.h>

using namespace lidar_od;
using namespace lidar_od::test;

namespace {

// Points on the two visible sides of a 4.5 x 1.8 m car seen from the sensor,
// i.e. the L-shape a scanner actually sees, rotated by `yaw`.
Cloud lShape(float yaw, const Eigen::Vector2f& centre, Indices& idx) {
    Cloud cloud;
    std::mt19937 rng(11);
    std::normal_distribution<float> noise(0.0f, 0.02f);
    const Eigen::Rotation2Df R(yaw);
    auto add = [&](float a, float b) {
        for (float z : {-1.4f, -1.0f, -0.6f}) {
            const Eigen::Vector2f p = centre + R * Eigen::Vector2f(a + noise(rng), b + noise(rng));
            cloud.push_back(makePoint(p.x(), p.y(), z));
        }
    };
    for (float a = -2.25f; a <= 2.25f; a += 0.1f) add(a, -0.9f);  // long side facing us
    for (float b = -0.9f; b <= 0.9f; b += 0.1f) add(-2.25f, b);   // rear
    idx.resize(cloud.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = static_cast<int>(i);
    return cloud;
}

float headingError(float a, float b) { return std::abs(wrapHalfPi(a - b)); }

class LShapeTest : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST_P(LShapeTest, RecoversHeadingAndSizeOfPartialView) {
    BoxParams params;
    params.method = "lshape";
    params.criterion = GetParam();
    const auto fitter = makeBoxFitter(params);
    for (float deg : {0.0f, 17.0f, 33.0f, 61.0f, 80.0f, 125.0f}) {
        const float yaw = deg * static_cast<float>(M_PI) / 180.0f;
        Indices idx;
        const Cloud cloud = lShape(yaw, Eigen::Vector2f(12.0f, 4.0f), idx);
        const OrientedBox b = fitter->fit(cloud, idx, -1.73f);
        EXPECT_LT(headingError(b.yaw, yaw), 2.5f * static_cast<float>(M_PI) / 180.0f)
            << GetParam() << " at " << deg << " deg";
        EXPECT_NEAR(b.size.x(), 4.5f, 0.15f);
        EXPECT_NEAR(b.size.y(), 1.8f, 0.15f);
        EXPECT_NEAR(b.center.x(), 12.0f, 0.15f);
        EXPECT_NEAR(b.center.y(), 4.0f, 0.15f);
    }
}

// The area criterion is left out on purpose: the convex hull of an L-shape is a
// right triangle, whose minimum-area rectangle is equally small when aligned
// with the legs or with the hypotenuse, so its heading is decided by noise.
INSTANTIATE_TEST_SUITE_P(Criteria, LShapeTest, ::testing::Values("closeness", "variance"));

TEST(BoxFitter, PcaIsBiasedOnPartialViewsWhereLShapeIsNot) {
    const float yaw = 30.0f * static_cast<float>(M_PI) / 180.0f;
    Indices idx;
    const Cloud cloud = lShape(yaw, Eigen::Vector2f(10.0f, 0.0f), idx);
    BoxParams params;
    params.method = "pca";
    const float pca_err = headingError(makeBoxFitter(params)->fit(cloud, idx, -1.73f).yaw, yaw);
    params.method = "lshape";
    const float l_err = headingError(makeBoxFitter(params)->fit(cloud, idx, -1.73f).yaw, yaw);
    EXPECT_GT(pca_err, 3.0f * static_cast<float>(M_PI) / 180.0f);
    EXPECT_LT(l_err, pca_err);
}

TEST(BoxFitter, SnapsBottomToNearbyGround) {
    Indices idx;
    const Cloud cloud = lShape(0.0f, Eigen::Vector2f(10.0f, 0.0f), idx);  // lowest point z = -1.4
    BoxParams params;
    const auto fitter = makeBoxFitter(params);
    const OrientedBox snapped = fitter->fit(cloud, idx, -1.73f);
    EXPECT_NEAR(snapped.center.z() - 0.5f * snapped.size.z(), -1.73f, 1e-4f);
    const OrientedBox floating = fitter->fit(cloud, idx, -2.5f);  // ground too far below
    EXPECT_NEAR(floating.center.z() - 0.5f * floating.size.z(), -1.4f, 0.05f);
}

TEST(Angles, WrapHelpers) {
    EXPECT_NEAR(wrapAngle(3.0f * static_cast<float>(M_PI)), static_cast<float>(M_PI), 1e-5f);
    EXPECT_NEAR(wrapHalfPi(static_cast<float>(M_PI)), 0.0f, 1e-5f);
    EXPECT_NEAR(wrapHalfPi(0.75f * static_cast<float>(M_PI)), -0.25f * static_cast<float>(M_PI), 1e-5f);
}
