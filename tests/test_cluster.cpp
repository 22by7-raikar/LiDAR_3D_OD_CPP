#include "lidar_od/cluster/clusterer.h"
#include "lidar_od/scan_layout.h"
#include "synthetic.h"

#include <gtest/gtest.h>

#include <set>

using namespace lidar_od;
using namespace lidar_od::test;

namespace {

// Objects only: indices of all returns that hit a box, plus which box was hit.
struct Objects {
    Cloud cloud;
    Indices points;
};

Objects scanObjects(const Scene& scene) {
    Objects o;
    std::vector<int> truth;
    o.cloud = scene.scan(truth);
    for (size_t i = 0; i < truth.size(); ++i)
        if (truth[i]) o.points.push_back(static_cast<int>(i));
    return o;
}

Scene threeObjects() {
    Scene s;
    s.boxes = {{6.0f, 10.5f, 2.0f, 3.8f, -0.2f},
               {6.0f, 10.5f, -3.8f, -2.0f, -0.2f},
               {14.0f, 14.6f, -0.3f, 0.3f, 0.1f}};
    return s;
}

class ClusterTest : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST_P(ClusterTest, FindsSeparatedObjects) {
    const Objects o = scanObjects(threeObjects());
    ClusterParams params;
    SensorParams sensor;
    sensor.columns = 900;  // matches the synthetic scanner
    const ScanLayout layout = computeScanLayout(o.cloud, sensor, true);
    ASSERT_TRUE(layout.from_ring_order);

    auto clusterer = makeClusterer(GetParam(), params, sensor);
    const auto clusters = clusterer->cluster(o.cloud, &layout, o.points);
    ASSERT_EQ(clusters.size(), 3u) << GetParam();

    // Every cluster must be pure: all of its points come from one box.
    for (const auto& c : clusters) {
        std::set<int> boxes;
        for (int i : c) boxes.insert(o.cloud[i].y > 1.0f ? 0 : (o.cloud[i].y < -1.0f ? 1 : 2));
        EXPECT_EQ(boxes.size(), 1u);
    }
}

INSTANTIATE_TEST_SUITE_P(AllMethods, ClusterTest, ::testing::Values("range_image", "euclidean"));

TEST(RangeImageCluster, SplitsObjectsAtDepthJumpEvenWhenAdjacentInImage) {
    // A pole right in front of a wall: neighbouring pixels, very different depth.
    // (The pole's shadow also cuts the wall in two, so the wall may be 2 clusters.)
    Scene s;
    s.boxes = {{8.0f, 8.3f, -0.15f, 0.15f, 0.5f}, {15.0f, 15.4f, -4.0f, 4.0f, 1.0f}};
    const Objects o = scanObjects(s);
    SensorParams sensor;
    sensor.columns = 900;
    RangeImageClusterer clusterer(ClusterParams{}, sensor);
    const auto clusters = clusterer.cluster(o.cloud, nullptr, o.points);

    int pole_clusters = 0;
    for (const auto& c : clusters) {
        int pole = 0;
        for (int i : c) pole += o.cloud[i].x < 10.0f;
        EXPECT_TRUE(pole == 0 || pole == static_cast<int>(c.size())) << "pole merged into wall";
        pole_clusters += pole > 0;
    }
    EXPECT_EQ(pole_clusters, 1);
}

TEST(ScanLayout, FallsBackToElevationWhenOrderIsScrambled) {
    const Objects o = scanObjects(threeObjects());
    Cloud shuffled = o.cloud;
    std::mt19937 rng(5);
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    SensorParams sensor;
    const ScanLayout layout = computeScanLayout(shuffled, sensor, true);
    EXPECT_FALSE(layout.from_ring_order);
    for (size_t i = 0; i < shuffled.size(); ++i) {
        EXPECT_GE(layout.row[i], 0);
        EXPECT_LT(layout.row[i], sensor.num_rings);
    }
}
