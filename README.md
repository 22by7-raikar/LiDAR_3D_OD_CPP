# LiDAR 3D Object Detection and Tracking (classical, C++)

Real-time 3D object detection and multi-object tracking on LiDAR point clouds,
**without deep learning**. Built in C++17 on PCL and Eigen.

![Tracking the cyclist and surrounding objects](docs/images/00_viewer_3d.png)

```
point cloud ─► ground segmentation ─► range-image clustering ─► L-shape boxes ─► Kalman + Hungarian tracking
                                                                    ▲                    ▲
                                                       local ground height      NDT ego-motion (world frame)
```

* **Ground segmentation:** four interchangeable methods. The default is
  region-wise plane fitting on a concentric zone model (Patchwork-style), with
  adaptive RANSAC, a ray-based slope walk and a grid minimum as alternatives.
* **Clustering:** range-image segmentation (Bogoslavskyi & Stachniss) on the
  exact laser rings recovered from scan order, about 4 ms per frame. Euclidean
  clustering on a balanced k-d tree is the alternative.
* **Boxes:** gravity-aligned L-shape fitting (Zhang et al.) that is robust to
  partial views, with the bottom snapped to the local ground and a filter that
  rejects walls, vegetation and noise.
* **Tracking:** a constant-velocity Kalman filter with χ² gating, a
  negative-log-likelihood cost, a matching cascade and Hungarian assignment,
  plus a tentative → confirmed → deleted life cycle. Tracks live in a **world
  frame** estimated by NDT LiDAR odometry, so speeds are absolute.
* **About 69 ms per frame** on one 2.1 GHz core, including odometry. Detection
  alone takes about 26 ms.

**[docs/ALGORITHMS.md](docs/ALGORITHMS.md)** explains every algorithm, the
alternatives, why each default was chosen, and the measured results, with
figures for each stage.

| Ground segmentation (4 methods) | Range image and clusters |
|---|---|
| ![](docs/images/03_ground_comparison.png) | ![](docs/images/02_range_image.png) |
| **Box fitting: AABB vs PCA vs L-shape** | **Tracks in the world frame** |
| ![](docs/images/06_box_fitting.png) | ![](docs/images/09_tracking_world.png) |

![Tracking animation](docs/images/tracking.gif)

## Build

Tested on Ubuntu 24.04 (PCL 1.14, GCC 13); any PCL >= 1.10 with C++17 should work:

```bash
sudo apt install cmake g++ libpcl-dev libyaml-cpp-dev libgtest-dev
cmake -S . -B build
cmake --build build -j
ctest --test-dir build          # 30 unit tests
```

The CMake options `LIDAR_OD_BUILD_VIEWER`, `LIDAR_OD_BUILD_TESTS` and
`LIDAR_OD_BUILD_APPS` (all `ON` by default) turn off parts you do not need.

## Run

```bash
# Interactive 3D viewer: space = pause, n = step, r = restart, v = camera, q = quit
./build/lidar_od_viewer --data data/sequence_02

# Headless: CSV detections, tracks, poses and per-stage timings, plus a timing summary
./build/lidar_od_run --data data/sequence_02 --out output/run

# Every algorithm on one frame, for comparisons and figures
./build/lidar_od_stages --data data/sequence_02 --frame 100 --out output/stages_100
python3 scripts/render_figures.py --stages output/stages_100 --run output/run \
    --data data/sequence_02 --out docs/images
```

All tools accept `--config FILE`. Every parameter lives in
[`config/default.yaml`](config/default.yaml). An override file needs only the
keys you change, and unknown keys are an error, so typos never pass silently:

```yaml
# my.yaml
ground:
  method: ransac
cluster:
  method: euclidean
```

## ROS 2

`ros2/lidar_od_ros` wraps the pipeline as a node. It subscribes to
`sensor_msgs/PointCloud2` and publishes ground and obstacle clouds, plus
detection and track markers for RViz:

```bash
cd ~/ros2_ws/src && ln -s /path/to/this/repo/ros2/lidar_od_ros .
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y && colcon build
ros2 launch lidar_od_ros lidar_od.launch.py points:=/velodyne_points
```

CI builds it against ROS 2 Jazzy.

## Layout

```
include/lidar_od/        public headers: config, pipeline, ground/, cluster/, box/, tracking/
src/                     implementations
apps/                    lidar_od_run, lidar_od_stages, lidar_od_viewer
tests/                   GoogleTest unit tests on synthetic ray-cast scenes and sample data
config/default.yaml      all parameters, documented
scripts/render_figures.py  figures for the docs
ros2/lidar_od_ros/       ROS 2 node and launch file
data/                    two sample sequences (see data/README.md for source and licence)
docs/                    ALGORITHMS.md and images
```

## Data

The sample sequences are KITTI raw scans (Velodyne HDL-64E, 10 Hz) under
CC BY-NC-SA 3.0. See [data/README.md](data/README.md) for attribution and for
how to run on your own PCD files.

## License

The code is released under the [MIT License](LICENSE). The sample data in
`data/` keeps its own licence (CC BY-NC-SA 3.0, see [data/README.md](data/README.md)).
