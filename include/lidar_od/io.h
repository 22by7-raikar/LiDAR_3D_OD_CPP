/// @file
/// @brief Reading PCD sequences from disk.
#pragma once

#include "lidar_od/types.h"

#include <string>
#include <vector>

namespace lidar_od {

/// @brief One frame of a sequence on disk.
struct SequenceFrame {
    std::string path;        ///< Path of the PCD file.
    double timestamp = 0.0;  ///< Acquisition time [s].
};

/// @brief Lists the PCD files of a directory in chronological order.
///
/// Files are sorted lexicographically. Timestamps are read from an optional
/// `timestamps.txt` in the same directory (one value in seconds per line, in
/// file order); without it, frames are spaced by @p frame_period.
/// @param directory Directory containing `.pcd` files.
/// @param frame_period Spacing used when there is no `timestamps.txt` [s].
/// @return The frames, in order.
/// @throws std::runtime_error If @p directory does not exist or the number of
///         timestamps does not match the number of files.
std::vector<SequenceFrame> listSequence(const std::string& directory, double frame_period);

/// @brief Loads one PCD file.
/// @param path File to read; needs at least x, y, z fields.
/// @return The cloud.
/// @throws std::runtime_error If the file cannot be read.
CloudPtr loadPcd(const std::string& path);

}  // namespace lidar_od
