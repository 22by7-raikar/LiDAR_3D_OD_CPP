#include "lidar_od/config.h"

#include <yaml-cpp/yaml.h>

#include <set>
#include <stdexcept>

namespace lidar_od {
namespace {

// Reads a YAML map into existing defaults and rejects keys nobody asked for,
// so a typo in a config file fails loudly instead of being silently ignored.
class Section {
public:
    // `node` may be absent (missing section) or null ("key:" with no value).
    Section(const YAML::Node& node, std::string path)
        : node_(node), present_(node.IsDefined() && !node.IsNull()), path_(std::move(path)) {
        if (present_ && !node_.IsMap())
            throw std::runtime_error("config: '" + path_ + "' must be a map");
    }

    ~Section() noexcept(false) {
        if (!present_ || std::uncaught_exceptions() > 0) return;
        for (const auto& kv : node_) {
            const auto key = kv.first.as<std::string>();
            if (!used_.count(key))
                throw std::runtime_error("config: unknown key '" + qualified(key) + "'");
        }
    }

    template <typename T>
    void get(const std::string& key, T& value) {
        used_.insert(key);
        const YAML::Node v = lookup(key);
        if (!v.IsDefined() || v.IsNull()) return;
        try {
            value = v.as<T>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error("config: bad value for '" + qualified(key) + "'");
        }
    }

    void get(const std::string& key, Eigen::Vector3f& value) {
        std::vector<float> v{value.x(), value.y(), value.z()};
        get(key, v);
        if (v.size() != 3)
            throw std::runtime_error("config: '" + qualified(key) + "' needs 3 values");
        value = Eigen::Vector3f(v[0], v[1], v[2]);
    }

    Section child(const std::string& key) {
        used_.insert(key);
        return Section(lookup(key), qualified(key));
    }

private:
    // Looks up through a const node: the non-const operator[] would insert the key.
    YAML::Node lookup(const std::string& key) const {
        if (!present_) return YAML::Node(YAML::NodeType::Undefined);
        const YAML::Node& self = node_;
        return self[key];
    }

    std::string qualified(const std::string& key) const {
        return path_.empty() ? key : path_ + "." + key;
    }

    YAML::Node node_;
    bool present_;
    std::string path_;
    std::set<std::string> used_;
};

}  // namespace

Config loadConfig(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("config: cannot read '" + path + "': " + e.what());
    }

    Config c;
    Section top(root, "");
    {
        auto s = top.child("sensor");
        s.get("height", c.sensor.height);
        s.get("num_rings", c.sensor.num_rings);
        s.get("fov_up_deg", c.sensor.fov_up_deg);
        s.get("fov_down_deg", c.sensor.fov_down_deg);
        s.get("columns", c.sensor.columns);
        s.get("frame_period", c.sensor.frame_period);
    }
    {
        auto s = top.child("preprocess");
        s.get("roi_min", c.preprocess.roi_min);
        s.get("roi_max", c.preprocess.roi_max);
        s.get("remove_ego", c.preprocess.remove_ego);
        s.get("ego_min", c.preprocess.ego_min);
        s.get("ego_max", c.preprocess.ego_max);
    }
    {
        auto g = top.child("ground");
        g.get("method", c.ground.method);
        {
            auto s = g.child("ransac");
            auto& p = c.ground.ransac;
            s.get("max_iterations", p.max_iterations);
            s.get("success_probability", p.success_probability);
            s.get("distance_threshold", p.distance_threshold);
            s.get("max_slope_deg", p.max_slope_deg);
            s.get("sample_max_height", p.sample_max_height);
            s.get("refine_iterations", p.refine_iterations);
            s.get("seed", p.seed);
        }
        {
            auto s = g.child("region");
            auto& p = c.ground.region;
            s.get("min_range", p.min_range);
            s.get("max_range", p.max_range);
            s.get("zone_edges", p.zone_edges);
            s.get("zone_rings", p.zone_rings);
            s.get("zone_sectors", p.zone_sectors);
            s.get("min_points_per_bin", p.min_points_per_bin);
            s.get("num_seed_points", p.num_seed_points);
            s.get("noise_floor", p.noise_floor);
            s.get("seed_threshold", p.seed_threshold);
            s.get("distance_threshold", p.distance_threshold);
            s.get("iterations", p.iterations);
            s.get("uprightness_deg", p.uprightness_deg);
            s.get("max_elevation", p.max_elevation);
            s.get("flatness_threshold", p.flatness_threshold);
            s.get("near_range_for_elevation", p.near_range_for_elevation);
        }
        {
            auto s = g.child("ray");
            auto& p = c.ground.ray;
            s.get("sector_deg", p.sector_deg);
            s.get("local_max_slope_deg", p.local_max_slope_deg);
            s.get("general_max_slope_deg", p.general_max_slope_deg);
            s.get("min_height_threshold", p.min_height_threshold);
            s.get("reclass_distance", p.reclass_distance);
            s.get("noise_floor", p.noise_floor);
        }
        {
            auto s = g.child("grid");
            auto& p = c.ground.grid;
            s.get("cell_size", p.cell_size);
            s.get("height_threshold", p.height_threshold);
            s.get("max_ground_height", p.max_ground_height);
            s.get("neighbourhood", p.neighbourhood);
            s.get("noise_floor", p.noise_floor);
        }
    }
    {
        auto s = top.child("cluster");
        s.get("method", c.cluster.method);
        s.get("min_points", c.cluster.min_points);
        s.get("max_points", c.cluster.max_points);
        {
            auto e = s.child("euclidean");
            e.get("voxel_leaf", c.cluster.euclidean.voxel_leaf);
            e.get("tolerance", c.cluster.euclidean.tolerance);
            e.get("tolerance_per_meter", c.cluster.euclidean.tolerance_per_meter);
        }
        {
            auto r = s.child("range_image");
            auto& p = c.cluster.range_image;
            r.get("angle_threshold_deg", p.angle_threshold_deg);
            r.get("max_col_gap", p.max_col_gap);
            r.get("max_row_gap", p.max_row_gap);
            r.get("max_distance", p.max_distance);
            r.get("use_ring_order", p.use_ring_order);
            r.get("merge_voxel", p.merge_voxel);
        }
    }
    {
        auto s = top.child("box");
        auto& p = c.box;
        s.get("method", p.method);
        s.get("criterion", p.criterion);
        s.get("angle_step_deg", p.angle_step_deg);
        s.get("ground_snap", p.ground_snap);
        s.get("min_height", p.min_height);
        s.get("max_height", p.max_height);
        s.get("max_length", p.max_length);
        s.get("max_width", p.max_width);
        s.get("max_bottom_above_ground", p.max_bottom_above_ground);
    }
    {
        auto s = top.child("ego_motion");
        auto& p = c.ego_motion;
        s.get("method", p.method);
        s.get("max_range", p.max_range);
        s.get("voxel_leaf", p.voxel_leaf);
        s.get("ndt_resolution", p.ndt_resolution);
        s.get("ndt_step_size", p.ndt_step_size);
        s.get("max_iterations", p.max_iterations);
        s.get("max_correspondence", p.max_correspondence);
    }
    {
        auto s = top.child("tracker");
        auto& p = c.tracker;
        {
            auto k = s.child("kalman");
            k.get("accel_noise", p.kalman.accel_noise);
            k.get("measurement_noise", p.kalman.measurement_noise);
            k.get("initial_velocity_sigma", p.kalman.initial_velocity_sigma);
        }
        s.get("gate_chi2", p.gate_chi2);
        s.get("max_distance", p.max_distance);
        s.get("confirm_hits", p.confirm_hits);
        s.get("max_missed_tentative", p.max_missed_tentative);
        s.get("max_missed_confirmed", p.max_missed_confirmed);
        s.get("size_smoothing", p.size_smoothing);
        s.get("history_length", p.history_length);
    }
    return c;
}

}  // namespace lidar_od
