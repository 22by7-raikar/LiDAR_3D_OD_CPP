/// @file
/// @brief Region-of-interest crop and voxel down-sampling on index sets.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/types.h"

namespace lidar_od {

/// @brief Keeps the points inside the region of interest and outside the ego vehicle.
///
/// Non-finite points are dropped. The result keeps the scan order of @p cloud.
/// @param cloud Raw cloud.
/// @param params Region of interest and ego-vehicle box.
/// @return Indices of the kept points.
Indices cropToRoi(const Cloud& cloud, const PreprocessParams& params);

/// @brief Voxel-grid down-sampling of a subset of a cloud.
///
/// Returns one existing point per occupied voxel, the one closest to the voxel
/// centroid, so the output still indexes real returns.
/// @param cloud Raw cloud.
/// @param input Subset to down-sample.
/// @param leaf Voxel edge length [m]; 0 or less returns @p input unchanged.
/// @return Indices of the representatives, in the order of @p input.
Indices voxelDownsample(const Cloud& cloud, const Indices& input, float leaf);

}  // namespace lidar_od
