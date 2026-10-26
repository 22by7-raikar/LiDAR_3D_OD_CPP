#include "lidar_od/tracking/kalman_filter.h"

#include <Eigen/Dense>

namespace lidar_od {

ConstantVelocityKalman::ConstantVelocityKalman(const KalmanParams& params, const Meas& first_position)
    : params_(params) {
    x_ << first_position, 0.0f, 0.0f;
    const float r2 = params_.measurement_noise * params_.measurement_noise;
    const float v2 = params_.initial_velocity_sigma * params_.initial_velocity_sigma;
    P_ = Eigen::Vector4f(r2, r2, v2, v2).asDiagonal();
    R_ = MeasCov::Identity() * r2;
}

void ConstantVelocityKalman::predict(float dt) {
    if (dt <= 0.0f) return;
    StateCov F = StateCov::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // Q = G G' sigma_a^2 with G = [dt^2/2, dt]' per axis.
    const float q = params_.accel_noise * params_.accel_noise;
    const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    StateCov Q = StateCov::Zero();
    Q(0, 0) = Q(1, 1) = dt4 / 4.0f * q;
    Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = dt3 / 2.0f * q;
    Q(2, 2) = Q(3, 3) = dt2 * q;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

ConstantVelocityKalman::MeasCov ConstantVelocityKalman::innovationCovariance() const {
    return P_.topLeftCorner<2, 2>() + R_;
}

float ConstantVelocityKalman::mahalanobis2(const Meas& z) const {
    const Meas y = innovation(z);
    return y.dot(innovationCovariance().ldlt().solve(y));
}

void ConstantVelocityKalman::update(const Meas& z) {
    // H = [I 0], so H P H' and P H' are just blocks of P.
    const MeasCov S = innovationCovariance();
    const Eigen::Matrix<float, 4, 2> PHt = P_.leftCols<2>();
    const Eigen::Matrix<float, 4, 2> K = PHt * S.inverse();
    x_ += K * innovation(z);

    // Joseph form keeps P symmetric positive definite despite rounding.
    Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
    H(0, 0) = H(1, 1) = 1.0f;
    const StateCov I_KH = StateCov::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
}

}  // namespace lidar_od
