#include "lidar_od/ground/ground_segmenter.h"
#include "plane_fit.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lidar_od {

RegionGroundSegmenter::RegionGroundSegmenter(const RegionGroundParams& params,
                                             const SensorParams& sensor)
    : params_(params), sensor_(sensor) {
    const size_t zones = params_.zone_rings.size();
    if (zones == 0 || params_.zone_sectors.size() != zones || params_.zone_edges.size() != zones + 1)
        throw std::invalid_argument("region ground: zone_edges needs one more entry than zone_rings/zone_sectors");
}

void RegionGroundSegmenter::segment(const Cloud& cloud, const Indices& input, Indices& ground,
                                    Indices& nonground) {
    ground.clear();
    nonground.clear();

    // Concentric zone model: wider rings and fewer sectors far away keep the
    // number of points per bin roughly balanced despite the falling density.
    const size_t zones = params_.zone_rings.size();
    std::vector<int> offset(zones + 1, 0);
    for (size_t z = 0; z < zones; ++z)
        offset[z + 1] = offset[z] + params_.zone_rings[z] * params_.zone_sectors[z];
    std::vector<Indices> bins(offset[zones]);
    bins_.assign(offset[zones], BinStats{});
    for (size_t z = 0; z < zones; ++z)
        for (int r = 0; r < params_.zone_rings[z]; ++r)
            for (int s = 0; s < params_.zone_sectors[z]; ++s) {
                auto& b = bins_[offset[z] + r * params_.zone_sectors[z] + s];
                b.zone = static_cast<int>(z);
                b.ring = r;
                b.sector = s;
            }

    for (int i : input) {
        const Point& p = cloud[i];
        const float range = std::hypot(p.x, p.y);
        if (range < params_.zone_edges.front() || range >= params_.zone_edges.back()) {
            nonground.push_back(i);
            continue;
        }
        size_t z = 0;
        while (range >= params_.zone_edges[z + 1]) ++z;
        const float width = (params_.zone_edges[z + 1] - params_.zone_edges[z]) / params_.zone_rings[z];
        const int ring = std::min(static_cast<int>((range - params_.zone_edges[z]) / width),
                                  params_.zone_rings[z] - 1);
        float az = std::atan2(p.y, p.x);
        if (az < 0.0f) az += 2.0f * static_cast<float>(M_PI);
        const int sector = std::min(
            static_cast<int>(az / (2.0f * static_cast<float>(M_PI)) * params_.zone_sectors[z]),
            params_.zone_sectors[z] - 1);
        bins[offset[z] + ring * params_.zone_sectors[z] + sector].push_back(i);
    }

    const float min_normal_z = std::cos(params_.uprightness_deg * static_cast<float>(M_PI) / 180.0f);
    const float road = -sensor_.height;
    // Returns well below the road are reflections (puddles, glass, drain covers):
    // never obstacles, and never allowed to seed a plane.
    const float noise_floor = road - params_.noise_floor;

    for (size_t b = 0; b < bins.size(); ++b) {
        Indices& pts = bins[b];
        BinStats& stats = bins_[b];
        stats.points = static_cast<int>(pts.size());
        if (pts.empty()) continue;

        // Too sparse for a plane: fall back to a plain height test.
        if (static_cast<int>(pts.size()) < params_.min_points_per_bin) {
            for (int i : pts) (cloud[i].z < road + 0.3f ? ground : nonground).push_back(i);
            continue;
        }

        std::sort(pts.begin(), pts.end(), [&](int a, int c) { return cloud[a].z < cloud[c].z; });
        size_t first = 0;
        while (first < pts.size() && cloud[pts[first]].z < noise_floor) ground.push_back(pts[first++]);
        pts.erase(pts.begin(), pts.begin() + static_cast<long>(first));

        // Lowest point representative: mean of the lowest points. A plane needs
        // broad support, so if the points near that height are fewer than
        // num_seed_points they are an isolated low artefact: skip past them.
        Indices current;
        float lpr = 0.0f;
        for (size_t start = 0; start < pts.size(); ++start) {
            const size_t end = std::min(pts.size(), start + static_cast<size_t>(params_.num_seed_points));
            lpr = 0.0f;
            for (size_t k = start; k < end; ++k) lpr += cloud[pts[k]].z;
            lpr /= static_cast<float>(end - start);
            current.clear();
            for (size_t k = start; k < pts.size() && cloud[pts[k]].z < lpr + params_.seed_threshold; ++k)
                current.push_back(pts[k]);
            if (static_cast<int>(current.size()) >= params_.num_seed_points || end == pts.size()) break;
            start += current.size() - 1;  // drop the whole unsupported group
        }
        if (current.size() < 3) {
            nonground.insert(nonground.end(), pts.begin(), pts.end());
            continue;
        }

        PlaneFit fit;
        bool ok = false;
        for (int it = 0; it < params_.iterations; ++it) {
            if (!fitPlane(cloud, current, fit)) break;
            ok = true;
            current.clear();
            for (int i : pts)
                if (signedDistance(fit.plane, cloud[i]) < params_.distance_threshold) current.push_back(i);
        }
        if (!ok || current.size() < 3) {
            nonground.insert(nonground.end(), pts.begin(), pts.end());
            continue;
        }
        stats.plane = fit.plane;

        // Ground likelihood estimation: the fitted plane must be upright, and
        // near the sensor it must also sit at road height unless it is very flat
        // (a raised but flat sidewalk is fine, a car hood is not).
        bool accept = fit.plane.z() >= min_normal_z;
        const float bin_range = std::hypot(fit.centroid.x(), fit.centroid.y());
        if (accept && bin_range < params_.near_range_for_elevation &&
            fit.centroid.z() > road + params_.max_elevation)
            accept = fit.flatness < params_.flatness_threshold;
        stats.accepted = accept;

        if (!accept) {
            nonground.insert(nonground.end(), pts.begin(), pts.end());
            continue;
        }
        for (int i : pts)
            (signedDistance(fit.plane, cloud[i]) < params_.distance_threshold ? ground : nonground).push_back(i);
    }
}

}  // namespace lidar_od
