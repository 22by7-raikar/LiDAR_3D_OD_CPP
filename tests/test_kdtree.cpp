#include "lidar_od/cluster/kdtree.h"

#include <gtest/gtest.h>

#include <random>

using namespace lidar_od;

TEST(KdTree, RadiusSearchMatchesBruteForce) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-20.0f, 20.0f);
    std::vector<Eigen::Vector3f> pts(5000);
    for (auto& p : pts) p = Eigen::Vector3f(u(rng), u(rng), 0.1f * u(rng));
    const KdTree3 tree(pts);

    for (int q = 0; q < 200; ++q) {
        const Eigen::Vector3f query(u(rng), u(rng), 0.1f * u(rng));
        const float radius = 0.2f + 0.01f * static_cast<float>(q);
        std::vector<int> found;
        tree.radiusSearch(query, radius, found);
        std::sort(found.begin(), found.end());

        std::vector<int> expected;
        for (size_t i = 0; i < pts.size(); ++i)
            if ((pts[i] - query).norm() <= radius) expected.push_back(static_cast<int>(i));
        EXPECT_EQ(found, expected) << "query " << q;
    }
}

TEST(KdTree, HandlesDuplicatesAndTinyInputs) {
    const KdTree3 empty(std::vector<Eigen::Vector3f>{});
    std::vector<int> out;
    empty.radiusSearch(Eigen::Vector3f::Zero(), 1.0f, out);
    EXPECT_TRUE(out.empty());

    const KdTree3 dup(std::vector<Eigen::Vector3f>(50, Eigen::Vector3f(1, 2, 3)));
    dup.radiusSearch(Eigen::Vector3f(1, 2, 3), 0.01f, out);
    EXPECT_EQ(out.size(), 50u);
}
