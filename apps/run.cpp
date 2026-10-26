// Headless runner: processes a sequence, writes detections, tracks, poses and
// per-stage timings as CSV, and prints a timing summary.

#include "args.h"

#include "lidar_od/io.h"
#include "lidar_od/pipeline.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace lidar_od;

namespace {

void writeBox(std::ostream& os, const OrientedBox& b) {
    os << b.center.x() << ',' << b.center.y() << ',' << b.center.z() << ',' << b.size.x() << ','
       << b.size.y() << ',' << b.size.z() << ',' << b.yaw;
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[static_cast<size_t>(q * static_cast<double>(v.size() - 1) + 0.5)];
}

}  // namespace

int main(int argc, char** argv) {
    const Args args(argc, argv,
                    {{"data", "data/sequence_02"}, {"config", "config/default.yaml"},
                     {"out", "output"}, {"frames", "0"}},
                    "usage: lidar_od_run [--data DIR] [--config FILE] [--out DIR] [--frames N]\n"
                    "  --frames N   process only the first N frames (0 = all)");

    const Config config = loadConfig(args.str("config"));
    auto frames = listSequence(args.str("data"), config.sensor.frame_period);
    if (args.integer("frames") > 0 && static_cast<size_t>(args.integer("frames")) < frames.size())
        frames.resize(args.integer("frames"));
    if (frames.empty()) {
        std::cerr << "no .pcd files in " << args.str("data") << "\n";
        return 1;
    }

    const std::filesystem::path out = args.str("out");
    std::filesystem::create_directories(out);
    std::ofstream det(out / "detections.csv"), trk(out / "tracks.csv"), pose(out / "poses.csv"),
        tim(out / "timing.csv");
    for (auto* f : {&det, &trk, &pose, &tim}) *f << std::setprecision(6) << std::fixed;
    det << "frame,id,points,cx,cy,cz,length,width,height,yaw\n";
    trk << "frame,track,confirmed,detection,age,hits,misses,wx,wy,wz,length,width,height,wyaw,"
           "vx,vy,sx,sy,sz,syaw\n";
    pose << "frame,timestamp,x,y,z,yaw\n";
    tim << "frame,points,roi,ground,nonground,clusters,detections,preprocess_ms,ground_ms,"
           "cluster_ms,boxes_ms,ego_motion_ms,tracking_ms,total_ms\n";

    Pipeline pipeline(config);
    std::vector<std::vector<double>> stage(7);
    for (size_t f = 0; f < frames.size(); ++f) {
        const CloudPtr cloud = loadPcd(frames[f].path);
        const FrameResult r = pipeline.process(cloud, frames[f].timestamp);

        for (const auto& d : r.detections) {
            det << f << ',' << d.id << ',' << d.indices.size() << ',';
            writeBox(det, d.box);
            det << '\n';
        }
        for (const auto& t : r.tracks) {
            trk << f << ',' << t.id << ',' << t.confirmed << ',' << t.detection << ',' << t.age << ','
                << t.hits << ',' << t.misses << ',';
            writeBox(trk, t.box_world);
            trk << ',' << t.velocity.x() << ',' << t.velocity.y() << ',' << t.box_sensor.center.x() << ','
                << t.box_sensor.center.y() << ',' << t.box_sensor.center.z() << ',' << t.box_sensor.yaw
                << '\n';
        }
        const Eigen::Vector3f p = r.pose.translation();
        pose << f << ',' << r.timestamp << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
             << std::atan2(r.pose.linear()(1, 0), r.pose.linear()(0, 0)) << '\n';

        const auto& t = r.timing;
        const double ms[7] = {t.preprocess, t.ground, t.cluster, t.boxes, t.ego_motion, t.tracking, t.total};
        for (int k = 0; k < 7; ++k) stage[k].push_back(ms[k]);
        tim << f << ',' << cloud->size() << ',' << r.roi.size() << ',' << r.ground.size() << ','
            << r.nonground.size() << ',' << r.detections.size() + r.rejected.size() << ','
            << r.detections.size();
        for (double v : ms) tim << ',' << v;
        tim << '\n';

        const int confirmed = static_cast<int>(
            std::count_if(r.tracks.begin(), r.tracks.end(), [](const auto& s) { return s.confirmed; }));
        std::cout << "frame " << std::setw(3) << f << "  detections " << std::setw(3) << r.detections.size()
                  << "  confirmed tracks " << std::setw(3) << confirmed << "  " << std::fixed
                  << std::setprecision(1) << t.total << " ms\n";
    }

    const char* names[7] = {"preprocess", "ground", "cluster", "boxes", "ego_motion", "tracking", "total"};
    std::cout << "\nstage          mean ms   p50 ms   p95 ms\n";
    for (int k = 0; k < 7; ++k) {
        double mean = 0.0;
        for (double v : stage[k]) mean += v;
        mean /= static_cast<double>(stage[k].size());
        std::cout << std::left << std::setw(12) << names[k] << std::right << std::setw(10) << mean
                  << std::setw(9) << percentile(stage[k], 0.5) << std::setw(9) << percentile(stage[k], 0.95)
                  << '\n';
    }
    std::cout << "\nwrote " << out.string() << "/{detections,tracks,poses,timing}.csv\n";
    return 0;
}
