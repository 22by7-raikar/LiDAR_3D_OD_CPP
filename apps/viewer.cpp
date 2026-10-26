// Interactive 3D viewer: plays a sequence through the pipeline and shows ground,
// objects, oriented boxes, track IDs, speeds and trails.
//
// Keys: space = pause/resume, n = next frame while paused, r = restart,
//       v = cycle camera (chase / top-down / side), q = quit.
// With --screenshot FILE it runs up to --frame N, saves a PNG and exits.

#include "args.h"

#include "lidar_od/io.h"
#include "lidar_od/pipeline.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <cstdio>
#include <iomanip>
#include <sstream>

using namespace lidar_od;

namespace {

struct Rgb {
    double r, g, b;
};

// Stable, well separated colour per track id (golden-ratio hue walk).
Rgb trackColour(int id) {
    const double h = std::fmod(0.11 + 0.618033988749895 * id, 1.0) * 6.0;
    const double x = 1.0 - std::abs(std::fmod(h, 2.0) - 1.0);
    const double s = 0.85, v = 0.95, c = s * v, m = v - c;
    double r = 0, g = 0, b = 0;
    switch (static_cast<int>(h)) {
        case 0: r = c, g = x * c; break;
        case 1: r = x * c, g = c; break;
        case 2: g = c, b = x * c; break;
        case 3: g = x * c, b = c; break;
        case 4: r = x * c, b = c; break;
        default: r = c, b = x * c; break;
    }
    return {r + m, g + m, b + m};
}

void addCloud(pcl::visualization::PCLVisualizer& viewer, const Cloud& cloud, const Indices& idx,
              const std::string& name, const Rgb& colour, double size) {
    CloudPtr subset(new Cloud);
    subset->reserve(idx.size());
    for (int i : idx) subset->push_back(cloud[i]);
    viewer.addPointCloud<Point>(subset, name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g,
                                            colour.b, name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
}

void addBox(pcl::visualization::PCLVisualizer& viewer, const OrientedBox& b, const std::string& name,
            const Rgb& colour, double opacity) {
    const Eigen::Quaternionf q(Eigen::AngleAxisf(b.yaw, Eigen::Vector3f::UnitZ()));
    viewer.addCube(b.center, q, b.size.x(), b.size.y(), b.size.z(), name);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                       pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colour.r, colour.g, colour.b,
                                       name);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, name);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, name);
}

enum class Camera { Chase, TopDown, Side };

void setCamera(pcl::visualization::PCLVisualizer& viewer, Camera cam) {
    switch (cam) {
        case Camera::Chase: viewer.setCameraPosition(-16, -9, 11, 14, 0, -1.5, 0, 0, 1); break;
        case Camera::TopDown: viewer.setCameraPosition(10, 0, 55, 10, 0, 0, 1, 0, 0); break;
        case Camera::Side: viewer.setCameraPosition(10, -35, 6, 10, 0, 0, 0, 0, 1); break;
    }
}

void draw(pcl::visualization::PCLVisualizer& viewer, const FrameResult& r, int frame) {
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    const Cloud& cloud = *r.cloud;

    addCloud(viewer, cloud, r.ground, "ground", {0.35, 0.38, 0.42}, 1.0);
    std::vector<char> is_object(cloud.size(), 0);
    for (const auto& d : r.detections)
        for (int i : d.indices) is_object[i] = 1;
    Indices clutter;
    for (int i : r.nonground)
        if (!is_object[i]) clutter.push_back(i);
    addCloud(viewer, cloud, clutter, "clutter", {0.75, 0.75, 0.78}, 1.0);

    const Eigen::Isometry3f sensor_from_world = r.pose.inverse();
    int shown = 0;
    for (const auto& t : r.tracks) {
        if (!t.confirmed) continue;
        ++shown;
        const Rgb c = trackColour(t.id);
        const std::string id = std::to_string(t.id);
        // Label and highlight only moving objects; static clutter (poles,
        // parked cars, vegetation) is drawn faintly so the scene stays readable.
        const bool moving = t.velocity.norm() > 1.0f;
        if (t.detection >= 0)
            addCloud(viewer, cloud, r.detections[t.detection].indices, "obj" + id, c, moving ? 3.0 : 2.0);
        addBox(viewer, t.box_sensor, "box" + id, c, moving ? (t.misses > 0 ? 0.5 : 1.0) : 0.35);
        if (!moving) continue;

        std::ostringstream label;
        label << '#' << t.id << "  " << std::fixed << std::setprecision(1) << t.velocity.norm() * 3.6f
              << " km/h";
        pcl::PointXYZ at(t.box_sensor.center.x(), t.box_sensor.center.y(),
                         t.box_sensor.center.z() + 0.5f * t.box_sensor.size.z() + 0.4f);
        viewer.addText3D(label.str(), at, 0.35, c.r, c.g, c.b, "label" + id);

        for (size_t k = 1; k < t.history.size(); ++k) {
            auto toSensor = [&](const Eigen::Vector2f& w) {
                const Eigen::Vector3f s = sensor_from_world * Eigen::Vector3f(w.x(), w.y(), 0.0f);
                return pcl::PointXYZ(s.x(), s.y(), t.box_sensor.center.z() - 0.5f * t.box_sensor.size.z());
            };
            viewer.addLine(toSensor(t.history[k - 1]), toSensor(t.history[k]), c.r, c.g, c.b,
                           "trail" + id + "_" + std::to_string(k));
        }
    }

    std::ostringstream hud;
    hud << "frame " << frame << "   " << r.detections.size() << " detections   " << shown
        << " confirmed tracks   " << std::fixed << std::setprecision(1) << r.timing.total << " ms";
    viewer.addText(hud.str(), 12, 12, 14, 1, 1, 1, "hud");
}

}  // namespace

int main(int argc, char** argv) {
    const Args args(argc, argv,
                    {{"data", "data/sequence_02"}, {"config", "config/default.yaml"}, {"screenshot", ""},
                     {"frame", "0"}, {"camera", "chase"}, {"size", "1600x900"}},
                    "usage: lidar_od_viewer [--data DIR] [--config FILE]\n"
                    "                       [--screenshot FILE --frame N --camera chase|top|side --size WxH]\n"
                    "keys: space pause, n step, r restart, v camera, q quit");
    const Config config = loadConfig(args.str("config"));
    const auto frames = listSequence(args.str("data"), config.sensor.frame_period);
    if (frames.empty()) {
        std::cerr << "no .pcd files in " << args.str("data") << "\n";
        return 1;
    }

    pcl::visualization::PCLVisualizer viewer("lidar_od");
    viewer.setBackgroundColor(0.07, 0.08, 0.10);
    viewer.initCameraParameters();
    const std::string& cam = args.str("camera");
    Camera camera = cam == "top" ? Camera::TopDown : cam == "side" ? Camera::Side : Camera::Chase;
    setCamera(viewer, camera);

    if (!args.str("screenshot").empty()) {
        int w = 1600, h = 900;
        std::sscanf(args.str("size").c_str(), "%dx%d", &w, &h);
        viewer.setSize(w, h);
        const size_t last = std::min<size_t>(args.integer("frame"), frames.size() - 1);
        Pipeline pipeline(config);
        FrameResult r;
        for (size_t f = 0; f <= last; ++f) r = pipeline.process(loadPcd(frames[f].path), frames[f].timestamp);
        draw(viewer, r, static_cast<int>(last));
        viewer.spinOnce(100);
        viewer.saveScreenshot(args.str("screenshot"));
        std::cout << "saved " << args.str("screenshot") << "\n";
        return 0;
    }

    bool paused = false, step = false, restart = false, cycle = false;
    viewer.registerKeyboardCallback([&](const pcl::visualization::KeyboardEvent& e) {
        if (!e.keyDown()) return;
        const std::string k = e.getKeySym();
        if (k == "space") paused = !paused;
        if (k == "n") step = true;
        if (k == "r") restart = true;
        if (k == "v") cycle = true;
    });

    Pipeline pipeline(config);
    size_t f = 0;
    while (!viewer.wasStopped()) {
        if (restart || f == frames.size()) {
            f = 0;
            pipeline.reset();
            restart = false;
        }
        if (cycle) {
            camera = static_cast<Camera>((static_cast<int>(camera) + 1) % 3);
            setCamera(viewer, camera);
            cycle = false;
        }
        if (!paused || step) {
            const FrameResult r = pipeline.process(loadPcd(frames[f].path), frames[f].timestamp);
            draw(viewer, r, static_cast<int>(f));
            ++f;
            step = false;
        }
        viewer.spinOnce(static_cast<int>(config.sensor.frame_period * 1000.0));
    }
    return 0;
}
