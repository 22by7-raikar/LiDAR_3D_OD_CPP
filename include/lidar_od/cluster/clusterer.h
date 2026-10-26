#pragma once

#include "lidar_od/config.h"
#include "lidar_od/scan_layout.h"
#include "lidar_od/types.h"

#include <memory>
#include <string>

namespace lidar_od {

class Clusterer {
public:
    virtual ~Clusterer() = default;
    virtual std::string name() const = 0;

    // Groups `input` (non-ground indices into `cloud`) into objects. `layout`
    // describes the scan structure of `cloud` and may be null.
    virtual std::vector<Indices> cluster(const Cloud& cloud, const ScanLayout* layout,
                                         const Indices& input) = 0;
};

// method: range_image | euclidean
std::unique_ptr<Clusterer> makeClusterer(const std::string& method, const ClusterParams& params,
                                         const SensorParams& sensor);

// Region growing in 3D with a k-d tree; the radius grows with range because
// neighbouring beams drift apart the further away an object is.
class EuclideanClusterer : public Clusterer {
public:
    EuclideanClusterer(const ClusterParams& params);
    std::string name() const override { return "euclidean"; }
    std::vector<Indices> cluster(const Cloud& cloud, const ScanLayout* layout,
                                 const Indices& input) override;

private:
    ClusterParams params_;
};

// Bogoslavskyi & Stachniss, "Fast range image-based segmentation of sparse 3D
// laser scans for online operation" (IROS 2016): neighbouring pixels of the
// range image belong to one object when the angle between the farther beam
// and the line joining both points is large, i.e. there is no depth jump.
class RangeImageClusterer : public Clusterer {
public:
    RangeImageClusterer(const ClusterParams& params, const SensorParams& sensor);
    std::string name() const override { return "range_image"; }
    std::vector<Indices> cluster(const Cloud& cloud, const ScanLayout* layout,
                                 const Indices& input) override;

private:
    ClusterParams params_;
    SensorParams sensor_;
};

}  // namespace lidar_od
