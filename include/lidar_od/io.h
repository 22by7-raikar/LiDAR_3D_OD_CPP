#pragma once

#include "lidar_od/types.h"

#include <string>
#include <vector>

namespace lidar_od {

struct SequenceFrame {
    std::string path;
    double timestamp = 0.0;  // seconds
};

// Lists the .pcd files of a directory in lexicographic (= chronological) order.
// Timestamps come from an optional `timestamps.txt` (one value in seconds per line,
// same order as the files); otherwise frames are spaced by `frame_period`.
std::vector<SequenceFrame> listSequence(const std::string& directory, double frame_period);

CloudPtr loadPcd(const std::string& path);

}  // namespace lidar_od
