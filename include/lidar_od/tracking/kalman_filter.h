#pragma once

#include "lidar_od/config.h"

#include <Eigen/Core>

namespace lidar_od {

// Constant-velocity Kalman filter in the ground plane.
// State x = [px, py, vx, vy], measurement z = [px, py].
class ConstantVelocityKalman {
public:
    using State = Eigen::Vector4f;
    using StateCov = Eigen::Matrix4f;
    using Meas = Eigen::Vector2f;
    using MeasCov = Eigen::Matrix2f;

    ConstantVelocityKalman(const KalmanParams& params, const Meas& first_position);

    // Propagates the state by dt seconds. Process noise is the discretised
    // white-noise-acceleration model: an unknown acceleration with standard
    // deviation `accel_noise` acting over the interval.
    void predict(float dt);

    // Innovation y = z - Hx and its covariance S = HPH' + R for the current state.
    Meas innovation(const Meas& z) const { return z - x_.head<2>(); }
    MeasCov innovationCovariance() const;

    // Squared Mahalanobis distance of a measurement, used for gating.
    float mahalanobis2(const Meas& z) const;

    void update(const Meas& z);

    const State& state() const { return x_; }
    const StateCov& covariance() const { return P_; }
    Eigen::Vector2f position() const { return x_.head<2>(); }
    Eigen::Vector2f velocity() const { return x_.tail<2>(); }

private:
    KalmanParams params_;
    State x_;
    StateCov P_;
    MeasCov R_;
};

}  // namespace lidar_od
