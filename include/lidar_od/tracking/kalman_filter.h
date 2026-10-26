/// @file
/// @brief Constant-velocity Kalman filter in the ground plane.
#pragma once

#include "lidar_od/config.h"

#include <Eigen/Core>

namespace lidar_od {

/// @brief Constant-velocity Kalman filter in the ground plane.
///
/// State x = [px, py, vx, vy], measurement z = [px, py]. Process noise is the
/// discretised white-noise-acceleration model; the covariance update uses the
/// Joseph form so it stays symmetric positive definite.
class ConstantVelocityKalman {
public:
    using State = Eigen::Vector4f;     ///< [px, py, vx, vy].
    using StateCov = Eigen::Matrix4f;  ///< State covariance.
    using Meas = Eigen::Vector2f;      ///< [px, py].
    using MeasCov = Eigen::Matrix2f;   ///< Measurement or innovation covariance.

    /// @brief Starts a filter at a measured position with zero velocity.
    /// @param params Noise model.
    /// @param first_position First measurement [m].
    ConstantVelocityKalman(const KalmanParams& params, const Meas& first_position);

    /// @brief Propagates the state by @p dt seconds.
    ///
    /// Q = sigma_a² · G·Gᵀ per axis with G = [dt²/2, dt]ᵀ, where sigma_a is
    /// KalmanParams::accel_noise. Non-positive @p dt leaves the state unchanged.
    /// @param dt Time step [s].
    void predict(float dt);

    /// @brief Innovation y = z − Hx of a measurement.
    Meas innovation(const Meas& z) const { return z - x_.head<2>(); }

    /// @brief Innovation covariance S = HPHᵀ + R.
    MeasCov innovationCovariance() const;

    /// @brief Squared Mahalanobis distance yᵀ S⁻¹ y of a measurement, for gating.
    float mahalanobis2(const Meas& z) const;

    /// @brief Corrects the state with a position measurement.
    /// @param z Measured position [m].
    void update(const Meas& z);

    /// @brief Current state.
    const State& state() const { return x_; }
    /// @brief Current state covariance.
    const StateCov& covariance() const { return P_; }
    /// @brief Estimated position [m].
    Eigen::Vector2f position() const { return x_.head<2>(); }
    /// @brief Estimated velocity [m/s].
    Eigen::Vector2f velocity() const { return x_.tail<2>(); }

private:
    KalmanParams params_;
    State x_;
    StateCov P_;
    MeasCov R_;
};

}  // namespace lidar_od
