/// @file
/// @brief Ground segmentation: the common interface and four implementations.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace lidar_od {

/// @brief Splits points into ground and non-ground.
class GroundSegmenter {
public:
    virtual ~GroundSegmenter() = default;

    /// @brief Short method name, as used in the configuration.
    virtual std::string name() const = 0;

    /// @brief Classifies every point of @p input as ground or non-ground.
    /// @param cloud Raw cloud.
    /// @param input Points to classify (indices into @p cloud).
    /// @param[out] ground Ground points.
    /// @param[out] nonground All other points of @p input.
    virtual void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                         Indices& nonground) = 0;
};

/// @brief Creates a ground segmenter by name.
/// @param method `ransac`, `region`, `ray` or `grid`.
/// @param params Parameters of all methods.
/// @param sensor Sensor geometry (height above the road).
/// @return The segmenter.
/// @throws std::invalid_argument On an unknown method.
std::unique_ptr<GroundSegmenter> makeGroundSegmenter(const std::string& method,
                                                     const GroundParams& params,
                                                     const SensorParams& sensor);

/// @brief One global ground plane fitted with adaptive RANSAC.
///
/// Hypotheses are drawn only from points near the expected road height, planes
/// steeper than RansacGroundParams::max_slope_deg are rejected, the number of
/// iterations adapts to the best inlier ratio, and the winning plane is refined
/// by least squares. Points below the plane also count as ground.
///
/// Fast and robust on flat roads; a single plane cannot follow slopes or a
/// cambered road.
class RansacGroundSegmenter : public GroundSegmenter {
public:
    /// @param params Method parameters.
    /// @param sensor Sensor geometry.
    RansacGroundSegmenter(const RansacGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "ransac"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

    /// @brief Plane of the last call as (a, b, c, d) with c ≥ 0; zero if none was found.
    const Eigen::Vector4f& plane() const { return plane_; }
    /// @brief Number of RANSAC iterations of the last call.
    int iterationsUsed() const { return iterations_used_; }

private:
    RansacGroundParams params_;
    SensorParams sensor_;
    Eigen::Vector4f plane_ = Eigen::Vector4f::Zero();
    int iterations_used_ = 0;
};

/// @brief One plane per bin of a concentric zone model, after Patchwork
///        (Lim et al., RA-L 2021).
///
/// The surroundings are split into rings and sectors that grow with range. In
/// each bin a plane is fitted to seed points near the lowest returns and
/// refined iteratively. The bin is accepted as ground only if the plane is
/// upright and, close to the sensor, either near road height or very flat.
///
/// Follows slopes and uneven roads; costs more than the other methods and has
/// more parameters.
class RegionGroundSegmenter : public GroundSegmenter {
public:
    /// @param params Method parameters.
    /// @param sensor Sensor geometry.
    /// @throws std::invalid_argument If the zone vectors have inconsistent sizes.
    RegionGroundSegmenter(const RegionGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "region"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

    /// @brief Result for one bin of the zone model.
    struct BinStats {
        int zone = 0;    ///< Zone index.
        int ring = 0;    ///< Ring index within the zone.
        int sector = 0;  ///< Sector index within the zone.
        int points = 0;  ///< Points that fell in the bin.
        Eigen::Vector4f plane = Eigen::Vector4f::Zero();  ///< Fitted plane, zero if none.
        bool accepted = false;  ///< Whether the plane was accepted as ground.
    };
    /// @brief Per-bin results of the last call, for inspection and plots.
    const std::vector<BinStats>& lastBins() const { return bins_; }

private:
    RegionGroundParams params_;
    SensorParams sensor_;
    std::vector<BinStats> bins_;
};

/// @brief Walks each azimuth ray outwards and compares slopes.
///
/// Points are sorted by range within narrow azimuth sectors. A point is ground
/// if its slope from the previous ground point and its slope from the sensor's
/// ground point are both small.
///
/// Cheap and tolerant of gentle slopes; occlusion gaps and single noisy rays can
/// mislead it.
class RayGroundSegmenter : public GroundSegmenter {
public:
    /// @param params Method parameters.
    /// @param sensor Sensor geometry.
    RayGroundSegmenter(const RayGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "ray"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

private:
    RayGroundParams params_;
    SensorParams sensor_;
};

/// @brief Height above the lowest point of the surrounding grid cells.
///
/// The fastest method and a useful baseline. It needs a reference minimum in the
/// neighbourhood and has no notion of slope inside a cell.
class GridGroundSegmenter : public GroundSegmenter {
public:
    /// @param params Method parameters.
    /// @param sensor Sensor geometry.
    GridGroundSegmenter(const GridGroundParams& params, const SensorParams& sensor);
    std::string name() const override { return "grid"; }
    void segment(const Cloud& cloud, const Indices& input, Indices& ground,
                 Indices& nonground) override;

private:
    GridGroundParams params_;
    SensorParams sensor_;
};

/// @brief Coarse local ground elevation built from one frame's ground points.
///
/// Used to put box bottoms on the road and to reject floating clusters.
class GroundHeightMap {
public:
    /// @param cell_size Grid cell edge [m].
    /// @param default_height Height returned where no ground is known [m].
    GroundHeightMap(float cell_size, float default_height);

    /// @brief Rebuilds the map from ground points.
    /// @param cloud Raw cloud.
    /// @param ground Ground points (indices into @p cloud).
    void build(const Cloud& cloud, const Indices& ground);

    /// @brief Ground height at a location.
    ///
    /// Searches outwards up to three cells, because objects hide the ground
    /// directly beneath them.
    /// @param x Location x [m].
    /// @param y Location y [m].
    /// @return Mean ground height of the nearest occupied cells, or the default.
    float heightAt(float x, float y) const;

private:
    int64_t key(int ix, int iy) const { return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy); }
    float cell_;
    float default_;
    std::unordered_map<int64_t, std::pair<float, int>> cells_;  // sum z, count
};

}  // namespace lidar_od
