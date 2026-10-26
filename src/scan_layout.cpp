#include "lidar_od/scan_layout.h"

#include <algorithm>
#include <cmath>

namespace lidar_od {
namespace {

constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);

float azimuthDeg(const Point& p) {
    float az = std::atan2(p.y, p.x) * kRadToDeg;
    return az < 0.0f ? az + 360.0f : az;
}

bool valid(const Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           (p.x != 0.0f || p.y != 0.0f || p.z != 0.0f);
}

// Returns false if the cloud does not look like it is stored ring by ring.
bool ringsFromOrder(const Cloud& cloud, int num_rings, std::vector<int>& rows) {
    rows.assign(cloud.size(), -1);
    int ring = 0;
    float prev = -1.0f;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (!valid(cloud[i])) continue;
        const float az = azimuthDeg(cloud[i]);
        // A sweep restart shows up as a wrap from the end of the circle to its start.
        if (prev >= 0.0f && prev > 270.0f && az < 90.0f) ++ring;
        prev = az;
        rows[i] = ring;
    }
    const int found = ring + 1;
    // A scan may end with a few points that already belong to the next sweep.
    if (found < num_rings - num_rings / 10 || found > num_rings + 1) return false;
    for (int& r : rows)
        if (r >= num_rings) r = num_rings - 1;
    return true;
}

}  // namespace

ScanLayout computeScanLayout(const Cloud& cloud, const SensorParams& sensor, bool try_ring_order) {
    ScanLayout layout;
    layout.rows = sensor.num_rings;
    layout.cols = sensor.columns;
    layout.col.assign(cloud.size(), -1);

    layout.from_ring_order = try_ring_order && ringsFromOrder(cloud, sensor.num_rings, layout.row);
    if (!layout.from_ring_order) {
        layout.row.assign(cloud.size(), -1);
        const float span = sensor.fov_up_deg - sensor.fov_down_deg;
        for (size_t i = 0; i < cloud.size(); ++i) {
            const Point& p = cloud[i];
            if (!valid(p)) continue;
            const float el = std::atan2(p.z, std::hypot(p.x, p.y)) * kRadToDeg;
            const int r = static_cast<int>((sensor.fov_up_deg - el) / span * layout.rows);
            layout.row[i] = std::clamp(r, 0, layout.rows - 1);
        }
    }

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (layout.row[i] < 0) continue;
        const int c = static_cast<int>(azimuthDeg(cloud[i]) / 360.0f * layout.cols);
        layout.col[i] = std::min(c, layout.cols - 1);
    }
    return layout;
}

}  // namespace lidar_od
