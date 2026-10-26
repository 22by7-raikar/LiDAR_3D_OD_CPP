#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace lidar_od {

class GroundSegmenter {
public:
    virtual ~GroundSegmenter() = default;
    virtual std::string name() const = 0;

    // Splits `input` (indices into `cloud`) into ground and non-ground points.
    virtual void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                         Indices& nonground) = 0;
};

// method: ransac | region | ray | grid
std::unique_ptr<GroundSegmenter> makeGroundSegmenter(const std::string& method,
                                                     const GroundParams& params,
                                                     const SensorParams& sensor);

// Single dominant plane, fitted with adaptive RANSAC and refined by least squares.
class RansacGroundSegmenter : public GroundSegmenter {
public:
    RansacGroundSegmenter(const RansacGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "ransac"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

    // (a, b, c, d) of the last plane, normal pointing up; zero if none was found.
    const Eigen::Vector4f& plane() const { return plane_; }
    int iterationsUsed() const { return iterations_used_; }

private:
    RansacGroundParams params_;
    SensorParams sensor_;
    Eigen::Vector4f plane_ = Eigen::Vector4f::Zero();
    int iterations_used_ = 0;
};

// Region-wise plane fitting on a concentric zone model (after Patchwork / Patchwork++).
class RegionGroundSegmenter : public GroundSegmenter {
public:
    RegionGroundSegmenter(const RegionGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "region"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

    struct BinStats {
        int zone = 0, ring = 0, sector = 0;
        int points = 0;
        Eigen::Vector4f plane = Eigen::Vector4f::Zero();
        bool accepted = false;
    };
    const std::vector<BinStats>& lastBins() const { return bins_; }

private:
    RegionGroundParams params_;
    SensorParams sensor_;
    std::vector<BinStats> bins_;
};

// Walks each azimuth ray outwards and compares local and global slopes.
class RayGroundSegmenter : public GroundSegmenter {
public:
    RayGroundSegmenter(const RayGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "ray"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

private:
    RayGroundParams params_;
    SensorParams sensor_;
};

// Height above the lowest point of the surrounding grid cells.
class GridGroundSegmenter : public GroundSegmenter {
public:
    GridGroundSegmenter(const GridGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "grid"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

private:
    GridGroundParams params_;
    SensorParams sensor_;
};

// Coarse local ground elevation built from the ground points of a frame, used to
// put the bottom of boxes on the road and to reject floating clusters.
class GroundHeightMap {
public:
    GroundHeightMap(float cell_size, float default_height);
    void build(const Cloud& cloud, const Indices& ground);
    float heightAt(float x, float y) const;

private:
    int64_t key(int ix, int iy) const { return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy); }
    float cell_;
    float default_;
    std::unordered_map<int64_t, std::pair<float, int>> cells_;  // sum z, count
};

}  // namespace lidar_od
