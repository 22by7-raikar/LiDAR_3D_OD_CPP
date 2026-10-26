#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

#include <Eigen/Geometry>

namespace lidar_od {

// Frame-to-frame LiDAR odometry. Registers the static part of each scan
// (everything that was not detected as an object) to the previous one and
// chains the results into a pose of the sensor in a fixed world frame, so that
// tracks move with their true velocity instead of the velocity relative to us.
class EgoMotionEstimator {
public:
    explicit EgoMotionEstimator(const EgoMotionParams& params);

    // Returns T_world_sensor for this frame. `static_points` index into `cloud`.
    Eigen::Isometry3f update(const Cloud& cloud, const Indices& static_points);
    void reset();

    bool enabled() const { return params_.method != "none"; }
    bool lastConverged() const { return converged_; }
    double lastFitness() const { return fitness_; }

private:
    CloudPtr prepare(const Cloud& cloud, const Indices& static_points) const;

    EgoMotionParams params_;
    CloudPtr previous_;
    Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f last_delta_ = Eigen::Isometry3f::Identity();
    bool converged_ = true;
    double fitness_ = 0.0;
};

}  // namespace lidar_od
