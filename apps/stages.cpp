// Dumps every intermediate result of one frame, for every interchangeable
// algorithm, so the stages can be plotted and compared (scripts/render_figures.py).

#include "args.h"

#include "lidar_od/io.h"
#include "lidar_od/pipeline.h"
#include "lidar_od/preprocess.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace lidar_od;

namespace {

template <typename F>
double timeMs(F&& f, int repeats = 5) {
    double best = 1e30;
    for (int i = 0; i < repeats; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    }
    return best;
}

std::vector<int> labelsOf(size_t n, const std::vector<Indices>& clusters) {
    std::vector<int> label(n, -1);
    for (size_t c = 0; c < clusters.size(); ++c)
        for (int i : clusters[c]) label[i] = static_cast<int>(c);
    return label;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args(argc, argv,
                    {{"data", "data/sequence_02"}, {"config", "config/default.yaml"},
                     {"out", "output/stages"}, {"frame", "0"}},
                    "usage: lidar_od_stages [--data DIR] [--config FILE] [--out DIR] [--frame N]");

    const Config config = loadConfig(args.str("config"));
    const auto frames = listSequence(args.str("data"), config.sensor.frame_period);
    const int frame = args.integer("frame");
    if (frame < 0 || frame >= static_cast<int>(frames.size())) {
        std::cerr << "frame out of range (sequence has " << frames.size() << " frames)\n";
        return 1;
    }
    const std::filesystem::path out = args.str("out");
    std::filesystem::create_directories(out);
    std::ofstream stats(out / "stats.csv");
    stats << "stage,method,value,unit\n";

    const CloudPtr cloud = loadPcd(frames[frame].path);
    const size_t n = cloud->size();

    // Full pipeline result with the configured methods (final detections).
    Pipeline pipeline(config);
    const FrameResult result = pipeline.process(cloud, frames[frame].timestamp);
    const ScanLayout& layout = result.layout;
    const Indices& roi = result.roi;
    stats << "layout,ring_order," << layout.from_ring_order << ",bool\n";

    // Every ground segmenter on the same ROI.
    const std::vector<std::string> ground_methods{"ransac", "region", "ray", "grid"};
    std::vector<std::vector<int>> ground_label;
    for (const auto& m : ground_methods) {
        auto seg = makeGroundSegmenter(m, config.ground, config.sensor);
        Indices g, ng;
        const double ms = timeMs([&] { seg->segment(*cloud, roi, g, ng); });
        std::vector<int> label(n, -1);
        for (int i : g) label[i] = 1;
        for (int i : ng) label[i] = 0;
        ground_label.push_back(std::move(label));
        stats << "ground," << m << ',' << ms << ",ms\n";
        stats << "ground_points," << m << ',' << g.size() << ",count\n";
        if (auto* r = dynamic_cast<RansacGroundSegmenter*>(seg.get())) {
            stats << "ransac_iterations,ransac," << r->iterationsUsed() << ",count\n";
            const auto& p = r->plane();
            stats << "ransac_plane,ransac," << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << p.w() << ",abcd\n";
        }
        if (auto* r = dynamic_cast<RegionGroundSegmenter*>(seg.get())) {
            std::ofstream bins(out / "region_bins.csv");
            bins << "zone,ring,sector,points,accepted,nz\n";
            for (const auto& b : r->lastBins())
                bins << b.zone << ',' << b.ring << ',' << b.sector << ',' << b.points << ',' << b.accepted
                     << ',' << b.plane.z() << '\n';
        }
    }

    // Both clusterers on the configured ground segmentation.
    std::vector<std::vector<int>> cluster_label;
    for (const std::string m : {"range_image", "euclidean"}) {
        auto clusterer = makeClusterer(m, config.cluster, config.sensor);
        std::vector<Indices> clusters;
        const double ms = timeMs([&] { clusters = clusterer->cluster(*cloud, &layout, result.nonground); }, 3);
        cluster_label.push_back(labelsOf(n, clusters));
        stats << "cluster," << m << ',' << ms << ",ms\n";
        stats << "clusters," << m << ',' << clusters.size() << ",count\n";
    }

    // Every box fitter on the final detections.
    std::ofstream boxes(out / "boxes.csv");
    boxes << std::setprecision(6) << std::fixed;
    boxes << "method,detection,cx,cy,cz,length,width,height,yaw\n";
    BoxParams bp = config.box;
    GroundHeightMap ground_map(2.0f, -config.sensor.height);
    ground_map.build(*cloud, result.ground);
    for (const std::string m : {"aabb", "pca", "lshape"}) {
        bp.method = m;
        auto fitter = makeBoxFitter(bp);
        double total = 0.0;
        for (const auto& d : result.detections) {
            const Eigen::Vector3f c = d.box.center;
            OrientedBox b;
            total += timeMs([&] { b = fitter->fit(*cloud, d.indices, ground_map.heightAt(c.x(), c.y())); }, 1);
            boxes << m << ',' << d.id << ',' << b.center.x() << ',' << b.center.y() << ',' << b.center.z()
                  << ',' << b.size.x() << ',' << b.size.y() << ',' << b.size.z() << ',' << b.yaw << '\n';
        }
        stats << "boxes," << m << ',' << total << ",ms\n";
    }

    // L-shape criterion curves for every detection.
    std::ofstream scores(out / "lshape_scores.csv");
    scores << "detection,theta_deg,area,closeness,variance\n";
    const LShapeFitter area(LShapeFitter::Criterion::Area, 1.0f, 0.0f);
    const LShapeFitter close(LShapeFitter::Criterion::Closeness, 1.0f, 0.0f);
    const LShapeFitter var(LShapeFitter::Criterion::Variance, 1.0f, 0.0f);
    for (const auto& d : result.detections) {
        std::vector<Eigen::Vector2f> xy;
        for (int i : d.indices) xy.emplace_back(cloud->at(i).x, cloud->at(i).y);
        for (int deg = 0; deg < 90; ++deg) {
            const float t = static_cast<float>(deg) * static_cast<float>(M_PI) / 180.0f;
            scores << d.id << ',' << deg << ',' << area.score(xy, t) << ',' << close.score(xy, t) << ','
                   << var.score(xy, t) << '\n';
        }
    }

    std::vector<int> detection(n, -1), rejected(n, -1);
    for (const auto& d : result.detections)
        for (int i : d.indices) detection[i] = d.id;
    for (size_t k = 0; k < result.rejected.size(); ++k)
        for (int i : result.rejected[k].indices) rejected[i] = static_cast<int>(k);
    std::vector<char> in_roi(n, 0);
    for (int i : roi) in_roi[i] = 1;

    std::ofstream pts(out / "points.csv");
    pts << std::setprecision(4) << std::fixed;
    pts << "x,y,z,intensity,row,col,roi,ground_ransac,ground_region,ground_ray,ground_grid,"
           "cluster_range_image,cluster_euclidean,detection,rejected\n";
    for (size_t i = 0; i < n; ++i) {
        const Point& p = cloud->at(i);
        pts << p.x << ',' << p.y << ',' << p.z << ',' << p.intensity << ',' << layout.row[i] << ','
            << layout.col[i] << ',' << int(in_roi[i]);
        for (const auto& g : ground_label) pts << ',' << g[i];
        for (const auto& c : cluster_label) pts << ',' << c[i];
        pts << ',' << detection[i] << ',' << rejected[i] << '\n';
    }

    std::cout << "frame " << frame << ": " << n << " points, " << roi.size() << " in ROI, "
              << result.detections.size() << " detections (" << result.rejected.size()
              << " clusters rejected)\nwrote " << out.string() << "\n";
    return 0;
}
