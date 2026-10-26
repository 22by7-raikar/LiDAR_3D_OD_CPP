#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace lidar_od {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

// Indices always refer to the raw input cloud of the current frame, so every
// stage can be traced back to the original points (and their scan order).
using Indices = std::vector<int>;

// Gravity-aligned oriented box: rotation only about z.
// size = (length along yaw, width, height).
struct OrientedBox {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    float yaw = 0.0f;

    // Corners of the footprint, counter-clockwise, at z = 0.
    std::vector<Eigen::Vector2f> footprint() const;
};

struct Detection {
    int id = -1;
    OrientedBox box;  // sensor frame
    Indices indices;  // points of the cluster
};

// Wraps an angle to (-pi, pi].
float wrapAngle(float a);

// Wraps an angle to (-pi/2, pi/2]: box headings are only defined up to 180 degrees.
float wrapHalfPi(float a);

}  // namespace lidar_od
