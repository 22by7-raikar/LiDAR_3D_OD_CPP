#include "lidar_od/config.h"
#include "lidar_od/io.h"
#include "lidar_od/pipeline.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using namespace lidar_od;
namespace fs = std::filesystem;

namespace {

std::string writeTemp(const std::string& name, const std::string& text) {
    const fs::path p = fs::temp_directory_path() / name;
    std::ofstream(p) << text;
    return p.string();
}

}  // namespace

TEST(Config, DefaultFileLoadsAndMatchesBuiltInDefaults) {
    const Config c = loadConfig(std::string(LIDAR_OD_SOURCE_DIR) + "/config/default.yaml");
    const Config d;
    EXPECT_EQ(c.ground.method, d.ground.method);
    EXPECT_EQ(c.cluster.method, d.cluster.method);
    EXPECT_FLOAT_EQ(c.cluster.range_image.merge_voxel, d.cluster.range_image.merge_voxel);
    EXPECT_FLOAT_EQ(c.tracker.gate_chi2, d.tracker.gate_chi2);
    EXPECT_TRUE(c.preprocess.roi_min.isApprox(d.preprocess.roi_min));
    EXPECT_EQ(c.ground.region.zone_sectors, d.ground.region.zone_sectors);
}

TEST(Config, PartialOverrideKeepsOtherDefaults) {
    const Config c = loadConfig(writeTemp("lod_partial.yaml", "ground:\n  method: ray\n  ray:\n    sector_deg: 1.0\n"));
    EXPECT_EQ(c.ground.method, "ray");
    EXPECT_FLOAT_EQ(c.ground.ray.sector_deg, 1.0f);
    EXPECT_EQ(c.cluster.method, Config{}.cluster.method);
}

TEST(Config, RejectsTyposAndBadValues) {
    EXPECT_THROW(loadConfig(writeTemp("lod_typo.yaml", "ground:\n  methd: ray\n")), std::runtime_error);
    EXPECT_THROW(loadConfig(writeTemp("lod_bad.yaml", "sensor:\n  height: tall\n")), std::runtime_error);
    EXPECT_THROW(loadConfig(writeTemp("lod_vec.yaml", "preprocess:\n  roi_min: [1, 2]\n")), std::runtime_error);
    EXPECT_THROW(loadConfig("/nonexistent/config.yaml"), std::runtime_error);
}

TEST(Pipeline, UnknownMethodNamesFailAtConstruction) {
    Config c;
    c.ground.method = "magic";
    EXPECT_THROW(Pipeline{c}, std::invalid_argument);
}

TEST(Pipeline, RunsOnBundledSequence) {
    const std::string dir = std::string(LIDAR_OD_SOURCE_DIR) + "/data/sequence_01";
    if (!fs::is_directory(dir)) GTEST_SKIP() << "sample data not present";
    Config config;
    const auto frames = listSequence(dir, config.sensor.frame_period);
    ASSERT_GE(frames.size(), 3u);
    Pipeline pipeline(config);
    for (size_t f = 0; f < 3; ++f) {
        const FrameResult r = pipeline.process(loadPcd(frames[f].path), frames[f].timestamp);
        EXPECT_TRUE(r.layout.from_ring_order);
        EXPECT_GT(r.ground.size(), r.roi.size() / 3);
        EXPECT_FALSE(r.detections.empty());
        EXPECT_EQ(r.ground.size() + r.nonground.size(), r.roi.size());
    }
}
