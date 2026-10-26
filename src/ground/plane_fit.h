#pragma once

#include "lidar_od/types.h"

#include <Eigen/Eigenvalues>

namespace lidar_od {

struct PlaneFit {
    Eigen::Vector4f plane = Eigen::Vector4f::Zero();  // normal points up (c >= 0)
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    float flatness = 0.0f;  // smallest eigenvalue of the covariance
};

// Total least squares plane: normal = eigenvector of the smallest covariance eigenvalue.
inline bool fitPlane(const Cloud& cloud, const Indices& idx, PlaneFit& out) {
    if (idx.size() < 3) return false;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int i : idx) mean += cloud[i].getVector3fMap().cast<double>();
    mean /= static_cast<double>(idx.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i : idx) {
        const Eigen::Vector3d d = cloud[i].getVector3fMap().cast<double>() - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(idx.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d n = es.eigenvectors().col(0);  // eigenvalues are sorted ascending
    if (n.z() < 0.0) n = -n;
    out.centroid = mean.cast<float>();
    out.plane.head<3>() = n.cast<float>();
    out.plane.w() = static_cast<float>(-n.dot(mean));
    out.flatness = static_cast<float>(es.eigenvalues()(0));
    return true;
}

inline float signedDistance(const Eigen::Vector4f& plane, const Point& p) {
    return plane.x() * p.x + plane.y() * p.y + plane.z() * p.z + plane.w();
}

}  // namespace lidar_od
