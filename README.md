# LiDAR 3D Object Detection and Tracking

Classical (no deep learning) 3D object detection and multi-object tracking for
spinning LiDAR, in C++17 with PCL and Eigen, plus a ROS 2 node.

![Frame 100 in the 3D viewer](docs/images/00_viewer_3d.png)

| Stage | Default method | Alternatives |
|---|---|---|
| Ground segmentation | Region-wise planes on a concentric zone model | RANSAC plane, ray slope walk, grid minimum |
| Clustering | Range-image segmentation | Euclidean clustering on a k-d tree |
| Bounding boxes | L-shape fitting, gravity-aligned | PCA, axis-aligned |
| Ego-motion | NDT scan registration | GICP, ICP, none |
| Tracking | Constant-velocity Kalman filter, χ² gating, cascade Hungarian matching | |

About 73 ms per frame on one 2.1 GHz core including ego-motion, or 27 ms for
detection alone.

[docs/ALGORITHMS.md](docs/ALGORITHMS.md) explains each algorithm, compares the
alternatives with measurements, and gives the reasons for each default.

| Where the ground methods disagree | Clustering: range image vs Euclidean |
|---|---|
| ![](docs/images/03_ground_differences.png) | ![](docs/images/06_clustering.png) |
| **Box fitting: axis-aligned, PCA, L-shape** | **Tracks in the world frame** |
| ![](docs/images/07_box_fitting.png) | ![](docs/images/10_tracks.png) |

![Tracking animation](docs/images/tracking.gif)

## Build

Ubuntu 22.04 or 24.04 (both built and tested in CI):

```bash
sudo apt install cmake g++ libpcl-dev libyaml-cpp-dev libgtest-dev
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

The CMake options `LIDAR_OD_BUILD_APPS`, `LIDAR_OD_BUILD_VIEWER` and
`LIDAR_OD_BUILD_TESTS` (all `ON` by default) switch off parts you do not need.

**API reference:** install `doxygen`, then run `cmake --build build --target doc`
and open `build/doc/html/index.html`.

## Run

```bash
# 3D viewer. Keys: space pause, n step, r restart, v camera, q quit
./build/lidar_od_viewer --data data/sequence_02

# Headless: CSV detections, tracks, poses and per-stage timings
./build/lidar_od_run --data data/sequence_02 --out output/run

# Every method on one frame, for comparisons and the figures
./build/lidar_od_stages --data data/sequence_02 --frame 100 --out output/stages_100
```

All tools take `--config FILE`. Every parameter is listed in
[config/default.yaml](config/default.yaml). An override file only needs the keys
you change, and unknown keys are rejected:

```yaml
ground:
  method: ransac
cluster:
  method: euclidean
```

## Library use

```cpp
#include "lidar_od/io.h"
#include "lidar_od/pipeline.h"

lidar_od::Pipeline pipeline(lidar_od::loadConfig("config/default.yaml"));
for (const auto& frame : lidar_od::listSequence("data/sequence_02", 0.1)) {
    const auto result = pipeline.process(lidar_od::loadPcd(frame.path), frame.timestamp);
    for (const auto& track : result.tracks)
        if (track.confirmed) { /* track.id, track.box_world, track.velocity */ }
}
```

## ROS 2

`ros2/lidar_od_ros` runs the pipeline as a node. It subscribes to
`sensor_msgs/PointCloud2` and publishes ground and obstacle clouds plus
detection and track markers for RViz. It builds on Humble and Jazzy.

```bash
cd ~/ros2_ws/src && ln -s /path/to/repo/ros2/lidar_od_ros .
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y && colcon build
ros2 launch lidar_od_ros lidar_od.launch.py points:=/velodyne_points
```

## Layout

```
include/lidar_od/          public API (Doxygen-documented)
src/                       implementation
apps/                      lidar_od_run, lidar_od_stages, lidar_od_viewer
tests/                     unit tests on synthetic ray-cast scenes and sample data
config/default.yaml        all parameters
ros2/lidar_od_ros/         ROS 2 node and launch file
scripts/render_figures.py  figures in docs/images
data/                      two sample sequences
docs/                      algorithm notes, figures, API main page
```

## License

Code: [MIT](LICENSE). The sample scans in `data/` are from the KITTI dataset
and are licensed CC BY-NC-SA 3.0; see [data/README.md](data/README.md).
