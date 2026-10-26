#pragma once

#include "lidar_od/box/box_fitter.h"
#include "lidar_od/cluster/clusterer.h"
#include "lidar_od/config.h"
#include "lidar_od/ground/ground_segmenter.h"
#include "lidar_od/scan_layout.h"
#include "lidar_od/tracking/ego_motion.h"
#include "lidar_od/tracking/tracker.h"
#include "lidar_od/types.h"

#include <memory>

namespace lidar_od {

// Milliseconds spent in each stage of one frame.
struct StageTimings {
    double preprocess = 0, ground = 0, cluster = 0, boxes = 0, ego_motion = 0, tracking = 0, total = 0;
};

struct TrackSnapshot {
    int id = -1;
    bool confirmed = false;
    OrientedBox box_world;
    OrientedBox box_sensor;
    Eigen::Vector2f velocity = Eigen::Vector2f::Zero();  // world frame [m/s]
    int age = 0, hits = 0, misses = 0;
    int detection = -1;  // index into FrameResult::detections, -1 when coasting
    std::vector<Eigen::Vector2f> history;  // world frame
};

struct FrameResult {
    CloudConstPtr cloud;
    double timestamp = 0.0;
    ScanLayout layout;
    Indices roi;
    Indices ground;
    Indices nonground;
    std::vector<Detection> detections;       // passed the size / shape filter
    std::vector<Detection> rejected;         // clusters that did not
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();  // T_world_sensor
    std::vector<TrackSnapshot> tracks;
    StageTimings timing;
};

// Preprocess -> ground segmentation -> clustering -> box fitting -> ego-motion
// -> tracking. Each stage is chosen by name in the config.
class Pipeline {
public:
    explicit Pipeline(const Config& config);

    FrameResult process(const CloudConstPtr& cloud, double timestamp);
    void reset();

    const Config& config() const { return config_; }

private:
    bool keep(const OrientedBox& box, float ground_z) const;

    Config config_;
    std::unique_ptr<GroundSegmenter> ground_;
    std::unique_ptr<Clusterer> clusterer_;
    std::unique_ptr<BoxFitter> box_fitter_;
    EgoMotionEstimator ego_motion_;
    MultiObjectTracker tracker_;
};

// Transforms a gravity-aligned box with a pose whose rotation is (nearly) a yaw.
OrientedBox transformBox(const OrientedBox& box, const Eigen::Isometry3f& T);

}  // namespace lidar_od
