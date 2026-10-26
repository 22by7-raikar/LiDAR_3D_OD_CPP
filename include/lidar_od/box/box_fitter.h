#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

#include <memory>
#include <string>

namespace lidar_od {

class BoxFitter {
public:
    virtual ~BoxFitter() = default;
    virtual std::string name() const = 0;

    // Fits the footprint in the xy plane; height spans from `ground_z` (when the
    // cluster bottom is within `ground_snap` of it) to the highest point.
    OrientedBox fit(const Cloud& cloud, const Indices& cluster, float ground_z) const;

protected:
    explicit BoxFitter(float ground_snap) : ground_snap_(ground_snap) {}
    // Returns the heading of the footprint rectangle in [0, pi/2).
    virtual float heading(const std::vector<Eigen::Vector2f>& xy) const = 0;

private:
    float ground_snap_;
};

// method: lshape | pca | aabb
std::unique_ptr<BoxFitter> makeBoxFitter(const BoxParams& params);

// Axis-aligned box in the sensor frame, for reference only.
class AabbFitter : public BoxFitter {
public:
    explicit AabbFitter(float ground_snap) : BoxFitter(ground_snap) {}
    std::string name() const override { return "aabb"; }

protected:
    float heading(const std::vector<Eigen::Vector2f>&) const override { return 0.0f; }
};

// Heading from the principal axis of the xy covariance.
class PcaFitter : public BoxFitter {
public:
    explicit PcaFitter(float ground_snap) : BoxFitter(ground_snap) {}
    std::string name() const override { return "pca"; }

protected:
    float heading(const std::vector<Eigen::Vector2f>& xy) const override;
};

// Search-based rectangle fitting: Zhang et al., "Efficient L-Shape Fitting for
// Vehicle Detection Using Laser Scanners" (IV 2017). Every candidate heading in
// [0, 90) degrees is scored by how well the points hug two perpendicular edges.
class LShapeFitter : public BoxFitter {
public:
    enum class Criterion { Area, Closeness, Variance };
    LShapeFitter(Criterion criterion, float angle_step_deg, float ground_snap);
    std::string name() const override { return "lshape"; }

    static Criterion parseCriterion(const std::string& s);
    // Score of one heading (higher is better); exposed for tests and plots.
    float score(const std::vector<Eigen::Vector2f>& xy, float theta) const;

protected:
    float heading(const std::vector<Eigen::Vector2f>& xy) const override;

private:
    Criterion criterion_;
    float step_;
};

}  // namespace lidar_od
