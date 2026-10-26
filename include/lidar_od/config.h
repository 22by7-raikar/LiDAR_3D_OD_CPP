#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace lidar_od {

// All tunables of the pipeline. Defaults are tuned for a roof-mounted
// Velodyne HDL-64E (KITTI-style, ~1.73 m above the road, 10 Hz).
// Any field can be overridden from YAML, see config/default.yaml.

struct SensorParams {
    float height = 1.73f;  // sensor height above the road [m]
    int num_rings = 64;
    float fov_up_deg = 2.0f;       // used when rings cannot be recovered from point order
    float fov_down_deg = -24.9f;
    int columns = 2048;            // horizontal range-image resolution
    double frame_period = 0.1;     // [s] used when no timestamps are available
};

struct PreprocessParams {
    Eigen::Vector3f roi_min{-20.0f, -12.0f, -2.5f};
    Eigen::Vector3f roi_max{40.0f, 12.0f, 1.0f};
    bool remove_ego = true;
    Eigen::Vector3f ego_min{-1.5f, -1.7f, -2.0f};
    Eigen::Vector3f ego_max{2.6f, 1.7f, 0.5f};
};

struct RansacGroundParams {
    int max_iterations = 200;        // hard cap; adaptive stopping usually ends far earlier
    float success_probability = 0.999f;
    float distance_threshold = 0.2f;
    float max_slope_deg = 15.0f;     // reject planes whose normal is further than this from +z
    float sample_max_height = 0.6f;  // only sample hypotheses from points below -height + this
    int refine_iterations = 2;       // least-squares refits on the inlier set
    unsigned seed = 42;
};

struct RegionGroundParams {
    float min_range = 2.7f;
    float max_range = 80.0f;
    // Concentric zone model: zone k spans [zone_edges[k], zone_edges[k+1]).
    std::vector<float> zone_edges{2.7f, 12.0f, 22.0f, 40.0f, 80.0f};
    std::vector<int> zone_rings{2, 4, 4, 4};
    std::vector<int> zone_sectors{16, 32, 54, 32};
    int min_points_per_bin = 10;
    int num_seed_points = 20;     // lowest points averaged to pick seeds; also the minimum seed count
    float noise_floor = 0.5f;     // points this far below -height are reflections, not road
    float seed_threshold = 0.4f;  // seeds: z < lowest-mean + this
    float distance_threshold = 0.15f;
    int iterations = 3;
    float uprightness_deg = 45.0f;    // max normal tilt
    float max_elevation = 0.7f;       // bin mean height above -sensor height accepted unconditionally...
    float flatness_threshold = 0.002f;  // ...or, if higher, accepted only when this flat (smallest eigenvalue)
    float near_range_for_elevation = 22.0f;  // elevation/flatness tests only apply closer than this
};

struct RayGroundParams {
    float sector_deg = 0.4f;
    float local_max_slope_deg = 8.0f;
    float general_max_slope_deg = 5.0f;
    float min_height_threshold = 0.08f;
    float reclass_distance = 2.0f;  // after a gap this long, re-anchor to the general ground model
    float noise_floor = 0.5f;       // points this far below -height are reflections -> ground
};

struct GridGroundParams {
    float cell_size = 0.5f;
    float height_threshold = 0.18f;   // above neighbourhood minimum
    float max_ground_height = 0.6f;   // neighbourhood minimum must be below -height + this
    int neighbourhood = 1;            // cells in each direction (1 -> 3x3)
    float noise_floor = 0.5f;         // points this far below -height are ignored for cell minima
};

struct GroundParams {
    std::string method = "region";  // ransac | region | ray | grid
    RansacGroundParams ransac;
    RegionGroundParams region;
    RayGroundParams ray;
    GridGroundParams grid;
};

struct EuclideanClusterParams {
    float voxel_leaf = 0.15f;         // 0 disables downsampling before clustering
    float tolerance = 0.45f;          // at the sensor
    float tolerance_per_meter = 0.012f;  // grows with range as the scan gets sparser
};

struct RangeImageClusterParams {
    float angle_threshold_deg = 10.0f;  // beta threshold of Bogoslavskyi & Stachniss
    int max_col_gap = 3;   // look past this many empty pixels horizontally
    int max_row_gap = 2;   // ... and vertically
    float max_distance = 1.5f;  // safety gate on the Euclidean distance between neighbours
    bool use_ring_order = true; // recover rows from scan order when possible
    float merge_voxel = 0.4f;   // merge clusters sharing a voxel of this size; 0 disables
};

struct ClusterParams {
    std::string method = "range_image";  // range_image | euclidean
    int min_points = 8;
    int max_points = 30000;
    EuclideanClusterParams euclidean;
    RangeImageClusterParams range_image;
};

struct BoxParams {
    std::string method = "lshape";  // lshape | pca | aabb
    std::string criterion = "variance";  // lshape: area | closeness | variance
    float angle_step_deg = 1.0f;
    float ground_snap = 0.4f;  // extend the box down to local ground if within this gap
    // Detection filter: drops walls, buildings, vegetation and noise.
    float min_height = 0.3f;
    float max_height = 4.0f;
    float max_length = 12.0f;
    float max_width = 5.0f;
    float max_bottom_above_ground = 1.0f;  // rejects overhanging foliage, signs, bridges
};

struct EgoMotionParams {
    std::string method = "ndt";  // ndt | icp | gicp | none
    float max_range = 50.0f;
    float voxel_leaf = 0.5f;
    float ndt_resolution = 1.5f;
    float ndt_step_size = 0.2f;
    int max_iterations = 40;
    float max_correspondence = 1.5f;  // icp / gicp
};

struct KalmanParams {
    float accel_noise = 2.0f;       // white-noise acceleration sigma [m/s^2]
    float measurement_noise = 0.25f; // position measurement sigma [m]
    float initial_velocity_sigma = 5.0f;
};

struct TrackerParams {
    KalmanParams kalman;
    float gate_chi2 = 9.21f;      // 99% for 2 DoF
    float max_distance = 3.0f;    // hard Euclidean gate [m]
    int confirm_hits = 3;
    int max_missed_tentative = 1;
    int max_missed_confirmed = 5;
    float size_smoothing = 0.3f;  // EMA weight for new size/yaw measurements
    int history_length = 30;
};

struct Config {
    SensorParams sensor;
    PreprocessParams preprocess;
    GroundParams ground;
    ClusterParams cluster;
    BoxParams box;
    EgoMotionParams ego_motion;
    TrackerParams tracker;
};

// Loads a YAML file on top of the defaults; missing keys keep their default.
// Throws std::runtime_error on unreadable files or unknown keys.
Config loadConfig(const std::string& path);

}  // namespace lidar_od
