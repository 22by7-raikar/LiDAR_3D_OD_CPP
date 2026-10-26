#include "lidar_od/io.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace lidar_od {

std::vector<SequenceFrame> listSequence(const std::string& directory, double frame_period) {
    if (!fs::is_directory(directory))
        throw std::runtime_error("not a directory: " + directory);

    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(directory))
        if (entry.is_regular_file() && entry.path().extension() == ".pcd")
            files.push_back(entry.path().string());
    std::sort(files.begin(), files.end());

    std::vector<double> stamps;
    std::ifstream ts(fs::path(directory) / "timestamps.txt");
    for (double t; ts >> t;) stamps.push_back(t);
    if (!stamps.empty() && stamps.size() != files.size())
        throw std::runtime_error("timestamps.txt has " + std::to_string(stamps.size()) +
                                 " entries for " + std::to_string(files.size()) + " frames");

    std::vector<SequenceFrame> frames(files.size());
    for (size_t i = 0; i < files.size(); ++i) {
        frames[i].path = files[i];
        frames[i].timestamp = stamps.empty() ? static_cast<double>(i) * frame_period : stamps[i];
    }
    return frames;
}

CloudPtr loadPcd(const std::string& path) {
    CloudPtr cloud(new Cloud);
    if (pcl::io::loadPCDFile<Point>(path, *cloud) < 0)
        throw std::runtime_error("cannot read " + path);
    return cloud;
}

}  // namespace lidar_od
