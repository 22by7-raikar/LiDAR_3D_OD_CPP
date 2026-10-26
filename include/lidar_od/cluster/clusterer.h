/// @file
/// @brief Clustering of non-ground points into objects.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/scan_layout.h"
#include "lidar_od/types.h"

#include <memory>
#include <string>

namespace lidar_od {

/// @brief Groups non-ground points into objects.
class Clusterer {
public:
    virtual ~Clusterer() = default;

    /// @brief Short method name, as used in the configuration.
    virtual std::string name() const = 0;

    /// @brief Splits @p input into clusters.
    /// @param cloud Raw cloud.
    /// @param layout Range-image coordinates of @p cloud; may be null, in which
    ///        case methods that need it compute their own.
    /// @param input Non-ground points (indices into @p cloud).
    /// @return Clusters within ClusterParams::min_points and ClusterParams::max_points.
    virtual std::vector<Indices> cluster(const Cloud& cloud, const ScanLayout* layout,
                                         const Indices& input) = 0;
};

/// @brief Creates a clusterer by name.
/// @param method `range_image` or `euclidean`.
/// @param params Parameters of all methods.
/// @param sensor Sensor geometry (range-image size).
/// @return The clusterer.
/// @throws std::invalid_argument On an unknown method.
std::unique_ptr<Clusterer> makeClusterer(const std::string& method, const ClusterParams& params,
                                         const SensorParams& sensor);

/// @brief Region growing in 3D on a k-d tree.
///
/// Points are first voxel-down-sampled, then grown breadth-first with a search
/// radius that increases with range, because neighbouring beams drift apart
/// further from the sensor. Every original point finally takes the label of
/// its nearest representative.
///
/// Simple and sensor-agnostic; merges objects that are close in 3D, and is
/// about ten times slower than RangeImageClusterer on a 64-beam scan.
class EuclideanClusterer : public Clusterer {
public:
    /// @param params Clustering parameters.
    explicit EuclideanClusterer(const ClusterParams& params);
    std::string name() const override { return "euclidean"; }
    std::vector<Indices> cluster(const Cloud& cloud, const ScanLayout* layout,
                                 const Indices& input) override;

private:
    ClusterParams params_;
};

/// @brief Connected components in the range image, after Bogoslavskyi and
///        Stachniss (IROS 2016).
///
/// Two neighbouring returns at ranges d1 ≥ d2, whose beams are an angle alpha
/// apart, belong to the same object when
/// beta = atan2(d2·sin(alpha), d1 − d2·cos(alpha)) exceeds
/// RangeImageClusterParams::angle_threshold_deg. Beta stays large along a
/// continuous surface and collapses at the depth jump between objects.
/// Neighbours may be a few empty pixels away, and clusters that touch in 3D
/// (surfaces seen at a grazing angle) are merged afterwards.
///
/// Fast and good at separating objects at different depths; needs the scan
/// structure, and splits surfaces seen at a grazing angle unless the merge
/// step joins them.
class RangeImageClusterer : public Clusterer {
public:
    /// @param params Clustering parameters.
    /// @param sensor Sensor geometry (range-image size, field of view).
    RangeImageClusterer(const ClusterParams& params, const SensorParams& sensor);
    std::string name() const override { return "range_image"; }
    std::vector<Indices> cluster(const Cloud& cloud, const ScanLayout* layout,
                                 const Indices& input) override;

private:
    ClusterParams params_;
    SensorParams sensor_;
};

}  // namespace lidar_od
