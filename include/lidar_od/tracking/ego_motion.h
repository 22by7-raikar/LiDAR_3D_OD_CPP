/// @file
/// @brief LiDAR odometry for tracking in a fixed frame.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

#include <Eigen/Geometry>

namespace lidar_od {

/// @brief Frame-to-frame LiDAR odometry.
///
/// Registers the static part of each scan (everything not detected as an
/// object) to the previous scan with NDT, GICP or ICP, using the previous
/// motion as the initial guess, and chains the results into the sensor pose in
/// a world frame fixed at the first scan. Tracking in that frame gives object
/// velocities over ground rather than relative to the moving sensor.
class EgoMotionEstimator {
public:
    /// @param params Method and registration parameters.
    /// @throws std::invalid_argument On an unknown method.
    explicit EgoMotionEstimator(const EgoMotionParams& params);

    /// @brief Registers a new scan.
    /// @param cloud Raw cloud.
    /// @param static_points Points assumed not to move (indices into @p cloud).
    /// @return Sensor pose in the world frame (identity for the first scan, and
    ///         always when the method is `none`).
    Eigen::Isometry3f update(const Cloud& cloud, const Indices& static_points);

    /// @brief Forgets the previous scan and resets the pose to identity.
    void reset();

    /// @brief False when the method is `none`.
    bool enabled() const { return params_.method != "none"; }
    /// @brief Whether the last registration converged; if not, the motion prior was used.
    bool lastConverged() const { return converged_; }
    /// @brief Fitness score of the last registration (method-specific; lower is better).
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
