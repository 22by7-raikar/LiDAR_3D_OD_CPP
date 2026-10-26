/// @file
/// @brief Range-image coordinates (laser ring and azimuth column) of every point.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

namespace lidar_od {

/// @brief Position of every raw point in a rings × columns range image.
struct ScanLayout {
    int rows = 0;                  ///< Number of rows (laser rings).
    int cols = 0;                  ///< Number of azimuth columns.
    std::vector<int> row;          ///< Row of each raw point, -1 for invalid points.
    std::vector<int> col;          ///< Column of each raw point, -1 for invalid points.
    bool from_ring_order = false;  ///< True when rows are the real laser rings.
};

/// @brief Computes the range-image row and column of every point.
///
/// Columns bin the azimuth into SensorParams::columns.
///
/// Rows: a rotating multi-beam scanner whose driver stores points laser by
/// laser restarts the azimuth sweep at every new ring, so counting those
/// restarts gives the laser index of every point. This is tried first when
/// @p try_ring_order is set; if the number of restarts does not match
/// SensorParams::num_rings, rows fall back to binning the elevation angle over
/// the configured vertical field of view.
/// @param cloud Raw cloud, in the order the driver produced it.
/// @param sensor Ring count, field of view and column count.
/// @param try_ring_order Whether to try recovering rings from the point order.
/// @return Row and column per point.
ScanLayout computeScanLayout(const Cloud& cloud, const SensorParams& sensor, bool try_ring_order);

}  // namespace lidar_od
