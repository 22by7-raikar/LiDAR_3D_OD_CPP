#include "lidar_od/cluster/clusterer.h"

#include <stdexcept>

namespace lidar_od {

std::unique_ptr<Clusterer> makeClusterer(const std::string& method, const ClusterParams& params,
                                         const SensorParams& sensor) {
    if (method == "range_image") return std::make_unique<RangeImageClusterer>(params, sensor);
    if (method == "euclidean") return std::make_unique<EuclideanClusterer>(params);
    throw std::invalid_argument("unknown cluster method '" + method + "'");
}

}  // namespace lidar_od
