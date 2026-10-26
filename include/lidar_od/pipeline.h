/// @file
/// @brief The complete per-frame pipeline and its output.
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

/// @brief Time spent in each stage of one frame [ms].
struct StageTimings {
    double preprocess = 0;  ///< Scan layout and region-of-interest crop.
    double ground = 0;      ///< Ground segmentation.
    double cluster = 0;     ///< Clustering.
    double boxes = 0;       ///< Box fitting and detection filter.
    double ego_motion = 0;  ///< Ego-motion estimation.
    double tracking = 0;    ///< Tracking.
    double total = 0;       ///< Whole frame.
};

/// @brief State of one track after a frame.
struct TrackSnapshot {
    int id = -1;                                         ///< Track identity, stable over time.
    bool confirmed = false;                              ///< False while the track is tentative.
    OrientedBox box_world;                               ///< Box in the world frame.
    OrientedBox box_sensor;                              ///< Same box in the current sensor frame.
    Eigen::Vector2f velocity = Eigen::Vector2f::Zero();  ///< Ground-plane velocity, world frame [m/s].
    int age = 0;                                         ///< Frames since the track was created.
    int hits = 0;                                        ///< Frames with an associated detection.
    int misses = 0;                                      ///< Consecutive frames without one.
    int detection = -1;  ///< Index into FrameResult::detections, -1 when coasting.
    std::vector<Eigen::Vector2f> history;  ///< Recent positions, world frame, oldest first.
};

/// @brief Everything the pipeline produced for one frame.
struct FrameResult {
    CloudConstPtr cloud;      ///< Input cloud; all indices refer to it.
    double timestamp = 0.0;   ///< Frame time [s].
    ScanLayout layout;        ///< Range-image coordinates of every point.
    Indices roi;              ///< Points inside the region of interest.
    Indices ground;           ///< Ground points (subset of #roi).
    Indices nonground;        ///< Non-ground points (subset of #roi).
    std::vector<Detection> detections;  ///< Clusters that passed the detection filter.
    std::vector<Detection> rejected;    ///< Clusters that did not.
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();  ///< Sensor pose in the world frame.
    std::vector<TrackSnapshot> tracks;  ///< All live tracks, tentative and confirmed.
    StageTimings timing;                ///< Per-stage runtime.
};

/// @brief Runs all stages on a stream of frames.
///
/// Region of interest → ground segmentation → clustering → box fitting and
/// detection filter → ego-motion → tracking. The algorithm of each stage is
/// chosen by name in the Config. The pipeline keeps state between frames
/// (odometry and tracks), so frames must be passed in time order.
class Pipeline {
public:
    /// @brief Builds every stage from the configuration.
    /// @param config Parameters and method names.
    /// @throws std::invalid_argument On an unknown method name.
    explicit Pipeline(const Config& config);

    /// @brief Processes the next frame.
    /// @param cloud Raw cloud in the sensor frame, in driver order.
    /// @param timestamp Frame time [s]; must not decrease between calls.
    /// @return Intermediate and final results of every stage.
    FrameResult process(const CloudConstPtr& cloud, double timestamp);

    /// @brief Forgets the odometry and all tracks, e.g. when a sequence restarts.
    void reset();

    /// @brief The configuration the pipeline was built with.
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

/// @brief Applies a rigid transform to a gravity-aligned box.
///
/// Only the yaw of @p T is applied to the box heading, so @p T should be
/// close to a rotation about z (true for vehicle odometry on a road).
/// @param box Box to transform.
/// @param T Transform from the box's frame to the target frame.
/// @return The box in the target frame.
OrientedBox transformBox(const OrientedBox& box, const Eigen::Isometry3f& T);

}  // namespace lidar_od
