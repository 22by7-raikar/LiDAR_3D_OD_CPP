/// @file
/// @brief Point, cloud, index and box types shared by every stage.
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

/// @brief Classical LiDAR 3D object detection and tracking.
namespace lidar_od {

/// Point type of every cloud: position plus return intensity.
using Point = pcl::PointXYZI;
/// Point cloud of #Point.
using Cloud = pcl::PointCloud<Point>;
/// Shared pointer to a mutable #Cloud.
using CloudPtr = Cloud::Ptr;
/// Shared pointer to an immutable #Cloud.
using CloudConstPtr = Cloud::ConstPtr;

/// @brief Indices into the raw input cloud of the current frame.
///
/// Stages exchange indices rather than copies, so every result can be traced
/// back to the original return and its position in the scan order.
using Indices = std::vector<int>;

/// @brief Gravity-aligned oriented box: the only rotation is a yaw about z.
struct OrientedBox {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();  ///< Box centre [m].
    Eigen::Vector3f size = Eigen::Vector3f::Zero();    ///< Length (along #yaw), width, height [m].
    float yaw = 0.0f;                                  ///< Heading of the length axis [rad].

    /// @brief Corners of the footprint in the xy plane.
    /// @return Four corners, counter-clockwise.
    std::vector<Eigen::Vector2f> footprint() const;
};

/// @brief One detected object of a frame.
struct Detection {
    int id = -1;      ///< Index of the detection within its frame.
    OrientedBox box;  ///< Box in the sensor frame.
    Indices indices;  ///< Points of the object.
};

/// @brief Wraps an angle to (-pi, pi].
/// @param a Angle [rad].
/// @return Equivalent angle in (-pi, pi].
float wrapAngle(float a);

/// @brief Wraps an angle to (-pi/2, pi/2].
///
/// Box headings are only defined up to 180 degrees, so this is the range used
/// for OrientedBox::yaw.
/// @param a Angle [rad].
/// @return Equivalent heading in (-pi/2, pi/2].
float wrapHalfPi(float a);

}  // namespace lidar_od
