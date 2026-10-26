/// @file
/// @brief All tunable parameters of the pipeline and the YAML loader.
///
/// Defaults suit a roof-mounted 64-beam spinning LiDAR about 1.73 m above the
/// road, running at 10 Hz. Every field can be overridden from YAML; see
/// `config/default.yaml`, which lists them all with the same names.
#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace lidar_od {

/// @brief Sensor geometry.
struct SensorParams {
    float height = 1.73f;       ///< Sensor height above the road [m].
    int num_rings = 64;         ///< Number of lasers.
    float fov_up_deg = 2.0f;    ///< Upper edge of the vertical field of view [deg]; used for elevation binning.
    float fov_down_deg = -24.9f;  ///< Lower edge of the vertical field of view [deg]; used for elevation binning.
    int columns = 2048;         ///< Azimuth resolution of the range image.
    double frame_period = 0.1;  ///< Time between frames when no timestamps are given [s].
};

/// @brief Region of interest and ego-vehicle removal (sensor frame, metres).
struct PreprocessParams {
    Eigen::Vector3f roi_min{-20.0f, -12.0f, -2.5f};  ///< Lower corner of the region of interest.
    Eigen::Vector3f roi_max{40.0f, 12.0f, 1.0f};     ///< Upper corner of the region of interest.
    bool remove_ego = true;                          ///< Drop returns from the ego vehicle.
    Eigen::Vector3f ego_min{-1.5f, -1.7f, -2.0f};    ///< Lower corner of the ego-vehicle box.
    Eigen::Vector3f ego_max{2.6f, 1.7f, 0.5f};       ///< Upper corner of the ego-vehicle box.
};

/// @brief RansacGroundSegmenter parameters.
struct RansacGroundParams {
    int max_iterations = 200;            ///< Upper bound; adaptive stopping usually ends much earlier.
    float success_probability = 0.999f;  ///< Probability of drawing at least one all-inlier sample.
    float distance_threshold = 0.2f;     ///< Inlier distance to the plane [m].
    float max_slope_deg = 15.0f;         ///< Planes tilted more than this are rejected [deg].
    float sample_max_height = 0.6f;      ///< Hypotheses use only points below road height + this [m].
    int refine_iterations = 2;           ///< Least-squares refits on the inlier set.
    unsigned seed = 42;                  ///< Random seed, for reproducible results.
};

/// @brief RegionGroundSegmenter parameters.
struct RegionGroundParams {
    /// Zone boundaries [m]; zone k spans [zone_edges[k], zone_edges[k+1]).
    std::vector<float> zone_edges{2.7f, 12.0f, 22.0f, 40.0f, 80.0f};
    std::vector<int> zone_rings{2, 4, 4, 4};        ///< Radial divisions per zone.
    std::vector<int> zone_sectors{16, 32, 54, 32};  ///< Angular divisions per zone.
    int min_points_per_bin = 10;    ///< Sparser bins use a plain height test instead of a plane.
    int num_seed_points = 20;       ///< Lowest points averaged for the seed height; also the minimum seed count.
    float noise_floor = 0.5f;       ///< Points this far below the road are reflections: ground, never seeds [m].
    float seed_threshold = 0.4f;    ///< Seeds lie at most this far above the lowest-point average [m].
    float distance_threshold = 0.15f;  ///< Inlier distance to the bin's plane [m].
    int iterations = 3;             ///< Plane fit / inlier selection rounds.
    float uprightness_deg = 45.0f;  ///< Planes tilted more than this are rejected [deg].
    float max_elevation = 0.7f;     ///< Near planes higher than road + this must be very flat [m].
    float flatness_threshold = 0.002f;  ///< Smallest covariance eigenvalue that counts as very flat [m²].
    float near_range_for_elevation = 22.0f;  ///< Elevation and flatness tests apply only closer than this [m].
};

/// @brief RayGroundSegmenter parameters.
struct RayGroundParams {
    float sector_deg = 0.4f;             ///< Azimuth width of one ray [deg].
    float local_max_slope_deg = 8.0f;    ///< Maximum slope from the previous ground point [deg].
    float general_max_slope_deg = 5.0f;  ///< Maximum slope from the sensor's ground point [deg].
    float min_height_threshold = 0.08f;  ///< Height tolerance used for very short steps [m].
    float reclass_distance = 2.0f;       ///< After a gap this long, use only the global slope test [m].
    float noise_floor = 0.5f;            ///< Points this far below the road are reflections: ground [m].
};

/// @brief GridGroundSegmenter parameters.
struct GridGroundParams {
    float cell_size = 0.5f;          ///< Grid cell edge [m].
    float height_threshold = 0.18f;  ///< Ground points lie at most this far above the local minimum [m].
    float max_ground_height = 0.6f;  ///< The local minimum must be below road height + this [m].
    int neighbourhood = 1;           ///< Cells searched in each direction for the minimum (1 → 3×3).
    float noise_floor = 0.5f;        ///< Points this far below the road are ignored for minima [m].
};

/// @brief Ground segmentation method and its parameters.
struct GroundParams {
    std::string method = "region";  ///< `ransac`, `region`, `ray` or `grid`.
    RansacGroundParams ransac;      ///< Used when #method is `ransac`.
    RegionGroundParams region;      ///< Used when #method is `region`.
    RayGroundParams ray;            ///< Used when #method is `ray`.
    GridGroundParams grid;          ///< Used when #method is `grid`.
};

/// @brief EuclideanClusterer parameters.
struct EuclideanClusterParams {
    float voxel_leaf = 0.15f;            ///< Down-sampling before clustering [m]; 0 disables it.
    float tolerance = 0.45f;             ///< Neighbour radius at the sensor [m].
    float tolerance_per_meter = 0.012f;  ///< Radius growth per metre of range.
};

/// @brief RangeImageClusterer parameters.
struct RangeImageClusterParams {
    float angle_threshold_deg = 10.0f;  ///< Minimum angle beta between neighbours of one object [deg].
    int max_col_gap = 3;                ///< Empty pixels skipped when looking for a horizontal neighbour.
    int max_row_gap = 2;                ///< Empty pixels skipped when looking for a vertical neighbour.
    float max_distance = 1.5f;          ///< Neighbours further apart are never joined [m].
    bool use_ring_order = true;         ///< Recover rows from the point order when possible.
    float merge_voxel = 0.4f;           ///< Clusters sharing a voxel of this size are merged [m]; 0 disables.
};

/// @brief Clustering method, size limits and per-method parameters.
struct ClusterParams {
    std::string method = "range_image";   ///< `range_image` or `euclidean`.
    int min_points = 8;                   ///< Smaller clusters are discarded.
    int max_points = 30000;               ///< Larger clusters are discarded.
    EuclideanClusterParams euclidean;     ///< Used when #method is `euclidean`.
    RangeImageClusterParams range_image;  ///< Used when #method is `range_image`.
};

/// @brief Box fitting and the detection filter.
struct BoxParams {
    std::string method = "lshape";       ///< `lshape`, `pca` or `aabb`.
    std::string criterion = "variance";  ///< L-shape criterion: `area`, `closeness` or `variance`.
    float angle_step_deg = 1.0f;         ///< L-shape heading search step [deg].
    float ground_snap = 0.4f;            ///< Extend the box down to the ground across gaps up to this [m].
    float min_height = 0.3f;             ///< Detection filter: minimum box height [m].
    float max_height = 4.0f;             ///< Detection filter: maximum box height [m].
    float max_length = 12.0f;            ///< Detection filter: maximum box length [m].
    float max_width = 5.0f;              ///< Detection filter: maximum box width [m].
    float max_bottom_above_ground = 1.0f;  ///< Detection filter: box bottom at most this far above ground [m].
};

/// @brief EgoMotionEstimator parameters.
struct EgoMotionParams {
    std::string method = "ndt";       ///< `ndt`, `gicp`, `icp` or `none`.
    float max_range = 50.0f;          ///< Points further away are not registered [m].
    float voxel_leaf = 0.5f;          ///< Down-sampling before registration [m].
    float ndt_resolution = 1.5f;      ///< NDT cell size [m].
    float ndt_step_size = 0.2f;       ///< NDT line-search step [m].
    int max_iterations = 40;          ///< Registration iterations.
    float max_correspondence = 1.5f;  ///< ICP / GICP correspondence distance [m].
};

/// @brief ConstantVelocityKalman noise model.
struct KalmanParams {
    float accel_noise = 2.0f;             ///< White-noise acceleration, standard deviation [m/s²].
    float measurement_noise = 0.25f;      ///< Position measurement, standard deviation [m].
    float initial_velocity_sigma = 5.0f;  ///< Velocity standard deviation of a new track [m/s].
};

/// @brief MultiObjectTracker parameters.
struct TrackerParams {
    KalmanParams kalman;              ///< Motion and measurement noise.
    float gate_chi2 = 9.21f;          ///< Squared Mahalanobis gate (99 % of chi-square, 2 DoF).
    float max_distance = 3.0f;        ///< Euclidean gate [m].
    int confirm_hits = 3;             ///< Hits needed to confirm a tentative track.
    int max_missed_tentative = 1;     ///< Misses after which a tentative track is deleted.
    int max_missed_confirmed = 5;     ///< Misses after which a confirmed track is deleted.
    float size_smoothing = 0.3f;      ///< Weight of a new measurement in the size and yaw averages.
    int history_length = 30;          ///< Positions kept per track.
};

/// @brief Complete pipeline configuration.
struct Config {
    SensorParams sensor;          ///< Sensor geometry.
    PreprocessParams preprocess;  ///< Region of interest.
    GroundParams ground;          ///< Ground segmentation.
    ClusterParams cluster;        ///< Clustering.
    BoxParams box;                ///< Box fitting and detection filter.
    EgoMotionParams ego_motion;   ///< Ego-motion.
    TrackerParams tracker;        ///< Tracking.
};

/// @brief Loads a YAML configuration on top of the defaults.
///
/// Keys that are missing keep their default value. Unknown keys are an error,
/// so a misspelt parameter is reported instead of silently ignored.
/// @param path YAML file.
/// @return The configuration.
/// @throws std::runtime_error If the file cannot be read, contains an unknown
///         key, or a value has the wrong type.
Config loadConfig(const std::string& path);

}  // namespace lidar_od
