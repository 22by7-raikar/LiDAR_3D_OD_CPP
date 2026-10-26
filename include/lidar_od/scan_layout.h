#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

namespace lidar_od {

// Position of every raw point in a (rings x columns) range image.
struct ScanLayout {
    int rows = 0;
    int cols = 0;
    std::vector<int> row;  // per raw point, -1 if the point is invalid
    std::vector<int> col;
    bool from_ring_order = false;  // true when rows are the real laser rings
};

// Rotating multi-beam scanners that store points laser by laser (as KITTI raw
// does) restart the azimuth sweep at every new ring; counting those restarts
// recovers the exact laser index of every point. When the cloud is not stored
// that way, rows fall back to binning the elevation angle over the vertical FOV.
ScanLayout computeScanLayout(const Cloud& cloud, const SensorParams& sensor, bool try_ring_order);

}  // namespace lidar_od
