#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

namespace lidar_od {

// Region-of-interest crop plus removal of returns from the ego vehicle.
// Works on indices and keeps scan order, so later stages can still use it.
Indices cropToRoi(const Cloud& cloud, const PreprocessParams& params);

// Voxel-grid downsampling of a subset; returns one representative raw index per
// occupied voxel (the point closest to the voxel centroid), not synthetic points.
Indices voxelDownsample(const Cloud& cloud, const Indices& input, float leaf);

}  // namespace lidar_od
