#include "lidar_od/ground/ground_segmenter.h"
#include "lidar_od/preprocess.h"
#include "synthetic.h"

#include <gtest/gtest.h>

using namespace lidar_od;
using namespace lidar_od::test;

namespace {

Scene street() {
    Scene s;
    s.boxes = {{6.0f, 10.5f, 2.0f, 3.8f, -0.2f},     // car on the left
               {12.0f, 16.0f, -4.0f, -2.2f, -0.3f},  // car on the right
               {9.0f, 9.6f, -1.0f, -0.4f, 0.1f},     // pedestrian
               {-8.0f, -4.0f, 6.0f, 6.3f, 1.0f}};    // wall segment
    return s;
}

class GroundTest : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST_P(GroundTest, SeparatesFlatGroundFromObjects) {
    const Scene scene = street();
    std::vector<int> truth;
    const Cloud cloud = scene.scan(truth);
    PreprocessParams pre;
    pre.roi_min = Eigen::Vector3f(-30, -30, -3);
    pre.roi_max = Eigen::Vector3f(30, 30, 2);
    const Indices roi = cropToRoi(cloud, pre);

    SensorParams sensor;
    auto seg = makeGroundSegmenter(GetParam(), GroundParams{}, sensor);
    Indices ground, nonground;
    seg->segment(cloud, roi, ground, nonground);
    ASSERT_EQ(ground.size() + nonground.size(), roi.size());

    int tp = 0, fn = 0, fp = 0, objects = 0;
    for (int i : ground) (truth[i] ? fp : tp)++;
    for (int i : nonground) {
        if (truth[i]) ++objects;
        else ++fn;
    }
    const double ground_recall = static_cast<double>(tp) / (tp + fn);
    const double object_recall = static_cast<double>(objects) / (objects + fp);
    EXPECT_GT(ground_recall, 0.97) << GetParam();
    // The synthetic boxes reach down to the road, so their lowest ~0.2 m (the
    // distance threshold) legitimately ends up in the ground; nothing more may.
    EXPECT_GT(object_recall, 0.85) << GetParam();
}

TEST_P(GroundTest, BelowRoadReflectionsDoNotCorruptTheGround) {
    // A short arc of returns 0.7 m below the road next to the car, as produced
    // by a puddle or drain cover. It once seeded a tilted plane in its bin and
    // turned the road around it into an "obstacle".
    const Scene scene = street();
    std::vector<int> truth;
    Cloud cloud = scene.scan(truth);
    const size_t first_reflection = cloud.size();
    for (int k = 0; k < 15; ++k)
        cloud.push_back(makePoint(5.8f, 1.38f + 0.015f * k, -2.40f - 0.007f * k));
    truth.resize(cloud.size(), 0);

    Indices all(cloud.size());
    for (size_t i = 0; i < all.size(); ++i) all[i] = static_cast<int>(i);
    auto seg = makeGroundSegmenter(GetParam(), GroundParams{}, SensorParams{});
    Indices ground, nonground;
    seg->segment(cloud, all, ground, nonground);

    std::vector<char> is_ground(cloud.size(), 0);
    for (int i : ground) is_ground[i] = 1;
    int road_near = 0, road_near_ground = 0;
    for (size_t i = 0; i < first_reflection; ++i) {
        const Point& p = cloud[i];
        if (truth[i] || std::hypot(p.x, p.y) > 8.0f || p.x < 2.7f) continue;
        ++road_near;
        road_near_ground += is_ground[i];
    }
    EXPECT_GT(road_near_ground, 0.98 * road_near) << GetParam();
    for (size_t i = first_reflection; i < cloud.size(); ++i)
        EXPECT_TRUE(is_ground[i]) << GetParam() << ": reflection labelled as obstacle";
}

INSTANTIATE_TEST_SUITE_P(AllMethods, GroundTest, ::testing::Values("ransac", "region", "ray", "grid"));

TEST(RansacGround, RecoversTiltedPlane) {
    // Road rising 3 degrees along x, plus clutter above it.
    Cloud cloud;
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> u(-20.0f, 20.0f);
    const float slope = std::tan(3.0f * static_cast<float>(M_PI) / 180.0f);
    for (int i = 0; i < 4000; ++i) {
        const float x = u(rng), y = u(rng);
        cloud.push_back(makePoint(x, y, -1.73f + slope * x));
    }
    for (int i = 0; i < 1000; ++i) {
        const float x = u(rng), y = u(rng);
        cloud.push_back(makePoint(x, y, -1.0f + slope * x + 0.05f * std::abs(u(rng))));
    }
    Indices all(cloud.size());
    for (size_t i = 0; i < all.size(); ++i) all[i] = static_cast<int>(i);

    RansacGroundParams params;
    params.sample_max_height = 2.0f;  // the slope reaches above the default window
    RansacGroundSegmenter seg(params, SensorParams{});
    Indices ground, nonground;
    seg.segment(cloud, all, ground, nonground);

    const Eigen::Vector3f n = seg.plane().head<3>();
    const Eigen::Vector3f expected = Eigen::Vector3f(-slope, 0.0f, 1.0f).normalized();
    EXPECT_GT(n.dot(expected), 0.9999f);
    EXPECT_EQ(ground.size(), 4000u);
    EXPECT_LT(seg.iterationsUsed(), params.max_iterations);  // adaptive stopping kicked in
}

TEST(GroundHeightMap, FallsBackToNeighboursAndDefault) {
    Cloud cloud;
    cloud.push_back(makePoint(0.5f, 0.5f, -1.5f));
    GroundHeightMap map(1.0f, -1.73f);
    map.build(cloud, {0});
    EXPECT_FLOAT_EQ(map.heightAt(0.2f, 0.9f), -1.5f);
    EXPECT_FLOAT_EQ(map.heightAt(2.5f, 0.5f), -1.5f);   // two cells away
    EXPECT_FLOAT_EQ(map.heightAt(50.0f, 0.0f), -1.73f);  // nothing nearby
}
