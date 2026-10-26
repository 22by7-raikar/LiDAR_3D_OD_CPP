#include "lidar_od/pipeline.h"
#include "lidar_od/preprocess.h"

#include <chrono>
#include <cmath>

namespace lidar_od {
namespace {

class Stopwatch {
public:
    Stopwatch() : start_(std::chrono::steady_clock::now()) {}
    double lap() {
        const auto now = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(now - start_).count();
        start_ = now;
        return ms;
    }

private:
    std::chrono::steady_clock::time_point start_;
};

float yawOf(const Eigen::Isometry3f& T) {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
}

}  // namespace

OrientedBox transformBox(const OrientedBox& box, const Eigen::Isometry3f& T) {
    OrientedBox out = box;
    out.center = T * box.center;
    out.yaw = wrapHalfPi(box.yaw + yawOf(T));
    return out;
}

Pipeline::Pipeline(const Config& config)
    : config_(config),
      ground_(makeGroundSegmenter(config.ground.method, config.ground, config.sensor)),
      clusterer_(makeClusterer(config.cluster.method, config.cluster, config.sensor)),
      box_fitter_(makeBoxFitter(config.box)),
      ego_motion_(config.ego_motion),
      tracker_(config.tracker) {}

void Pipeline::reset() {
    ego_motion_.reset();
    tracker_.reset();
}

bool Pipeline::keep(const OrientedBox& b, float ground_z) const {
    const auto& p = config_.box;
    const float bottom = b.center.z() - 0.5f * b.size.z();
    return b.size.z() >= p.min_height && b.size.z() <= p.max_height && b.size.x() <= p.max_length &&
           b.size.y() <= p.max_width && bottom - ground_z <= p.max_bottom_above_ground;
}

FrameResult Pipeline::process(const CloudConstPtr& cloud, double timestamp) {
    FrameResult r;
    r.cloud = cloud;
    r.timestamp = timestamp;
    Stopwatch total, sw;

    r.layout = computeScanLayout(*cloud, config_.sensor, config_.cluster.range_image.use_ring_order);
    r.roi = cropToRoi(*cloud, config_.preprocess);
    r.timing.preprocess = sw.lap();

    ground_->segment(*cloud, r.roi, r.ground, r.nonground);
    r.timing.ground = sw.lap();

    std::vector<Indices> clusters = clusterer_->cluster(*cloud, &r.layout, r.nonground);
    r.timing.cluster = sw.lap();

    GroundHeightMap ground_map(2.0f, -config_.sensor.height);
    ground_map.build(*cloud, r.ground);
    for (auto& c : clusters) {
        Detection d;
        Eigen::Vector2f mean = Eigen::Vector2f::Zero();
        for (int i : c) mean += Eigen::Vector2f(cloud->at(i).x, cloud->at(i).y);
        mean /= static_cast<float>(c.size());
        const float ground_z = ground_map.heightAt(mean.x(), mean.y());
        d.box = box_fitter_->fit(*cloud, c, ground_z);
        d.indices = std::move(c);
        if (keep(d.box, ground_z)) {
            d.id = static_cast<int>(r.detections.size());
            r.detections.push_back(std::move(d));
        } else {
            r.rejected.push_back(std::move(d));
        }
    }
    r.timing.boxes = sw.lap();

    // Odometry runs on everything that is not a detected object.
    std::vector<char> is_object(cloud->size(), 0);
    for (const auto& d : r.detections)
        for (int i : d.indices) is_object[i] = 1;
    Indices static_points;
    static_points.reserve(cloud->size());
    for (int i = 0; i < static_cast<int>(cloud->size()); ++i)
        if (!is_object[i] && std::isfinite(cloud->at(i).x)) static_points.push_back(i);
    r.pose = ego_motion_.update(*cloud, static_points);
    r.timing.ego_motion = sw.lap();

    std::vector<OrientedBox> world;
    world.reserve(r.detections.size());
    for (const auto& d : r.detections) world.push_back(transformBox(d.box, r.pose));
    tracker_.update(world, timestamp);

    const Eigen::Isometry3f sensor_from_world = r.pose.inverse();
    for (const Track& t : tracker_.tracks()) {
        TrackSnapshot s;
        s.id = t.id;
        s.confirmed = t.state == TrackState::Confirmed;
        s.box_world = t.box();
        s.box_sensor = transformBox(s.box_world, sensor_from_world);
        s.velocity = t.kf.velocity();
        s.age = t.age;
        s.hits = t.hits;
        s.misses = t.misses;
        s.detection = t.detection;
        s.history.assign(t.history.begin(), t.history.end());
        r.tracks.push_back(std::move(s));
    }
    r.timing.tracking = sw.lap();
    r.timing.total = total.lap();
    return r;
}

}  // namespace lidar_od
