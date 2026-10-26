/// @file
/// @brief Gravity-aligned bounding boxes for clusters.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

#include <memory>
#include <string>

namespace lidar_od {

/// @brief Fits a gravity-aligned box to a cluster.
///
/// Subclasses choose the heading; this class computes the extent along it,
/// orders length ≥ width, and places the box bottom on the ground when the
/// lowest point is close to it.
class BoxFitter {
public:
    virtual ~BoxFitter() = default;

    /// @brief Short method name, as used in the configuration.
    virtual std::string name() const = 0;

    /// @brief Fits the box.
    /// @param cloud Raw cloud.
    /// @param cluster Points of the object.
    /// @param ground_z Local ground height [m]; the box bottom snaps to it when
    ///        the lowest point is at most BoxParams::ground_snap above it.
    /// @return The box, sensor frame, yaw in (-pi/2, pi/2].
    OrientedBox fit(const Cloud& cloud, const Indices& cluster, float ground_z) const;

protected:
    /// @param ground_snap See BoxParams::ground_snap.
    explicit BoxFitter(float ground_snap) : ground_snap_(ground_snap) {}

    /// @brief Heading of the rectangle.
    /// @param xy Footprint points (at most a few hundred).
    /// @return Heading in [0, pi/2) [rad].
    virtual float heading(const std::vector<Eigen::Vector2f>& xy) const = 0;

private:
    float ground_snap_;
};

/// @brief Creates a box fitter from the configuration.
/// @param params BoxParams::method and its parameters.
/// @return The fitter.
/// @throws std::invalid_argument On an unknown method or criterion.
std::unique_ptr<BoxFitter> makeBoxFitter(const BoxParams& params);

/// @brief Axis-aligned box in the sensor frame; a baseline for comparison.
class AabbFitter : public BoxFitter {
public:
    /// @param ground_snap See BoxParams::ground_snap.
    explicit AabbFitter(float ground_snap) : BoxFitter(ground_snap) {}
    std::string name() const override { return "aabb"; }

protected:
    float heading(const std::vector<Eigen::Vector2f>&) const override { return 0.0f; }
};

/// @brief Heading from the principal axis of the footprint covariance.
///
/// Cheap and accurate when the whole outline is visible; biased on partial
/// (L-shaped) views, where the principal axis lies between the two sides.
class PcaFitter : public BoxFitter {
public:
    /// @param ground_snap See BoxParams::ground_snap.
    explicit PcaFitter(float ground_snap) : BoxFitter(ground_snap) {}
    std::string name() const override { return "pca"; }

protected:
    float heading(const std::vector<Eigen::Vector2f>& xy) const override;
};

/// @brief Search-based rectangle fitting, after Zhang et al. (IEEE IV 2017).
///
/// Every heading in [0°, 90°) is scored by how well the points fit the edges of
/// the bounding rectangle at that heading, and the best heading wins. Suited to
/// the partial L-shaped views a LiDAR gets of vehicles.
class LShapeFitter : public BoxFitter {
public:
    /// @brief How a candidate heading is scored.
    enum class Criterion {
        Area,       ///< Smaller rectangle is better. Ambiguous for L-shapes.
        Closeness,  ///< Points close to an edge are better. Sharp but noisy.
        Variance,   ///< Low spread of edge distances is better. Smooth.
    };

    /// @param criterion Scoring rule.
    /// @param angle_step_deg Search step [deg].
    /// @param ground_snap See BoxParams::ground_snap.
    /// @throws std::invalid_argument If @p angle_step_deg is not positive.
    LShapeFitter(Criterion criterion, float angle_step_deg, float ground_snap);
    std::string name() const override { return "lshape"; }

    /// @brief Parses `area`, `closeness` or `variance`.
    /// @throws std::invalid_argument On any other string.
    static Criterion parseCriterion(const std::string& s);

    /// @brief Score of one heading; higher is better.
    /// @param xy Footprint points.
    /// @param theta Candidate heading [rad].
    float score(const std::vector<Eigen::Vector2f>& xy, float theta) const;

protected:
    float heading(const std::vector<Eigen::Vector2f>& xy) const override;

private:
    Criterion criterion_;
    float step_;
};

}  // namespace lidar_od
