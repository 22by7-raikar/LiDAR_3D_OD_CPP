// ROS 2 wrapper around lidar_od::Pipeline.
//
// Subscribes:  points (sensor_msgs/PointCloud2, needs x y z, intensity optional)
// Publishes:   ~/ground, ~/obstacles   (sensor_msgs/PointCloud2)
//              ~/detections            (visualization_msgs/MarkerArray, this frame's boxes)
//              ~/tracks                (visualization_msgs/MarkerArray, confirmed tracks with ID and speed)
// Parameters:  config_file (string, YAML; empty = built-in defaults)
//
// Points are processed in the sensor frame of the incoming message. Drivers that
// do not store points ring by ring are handled automatically: the range image
// then bins rows by elevation angle (see ScanLayout).

#include "lidar_od/config.h"
#include "lidar_od/pipeline.h"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>

namespace {

using lidar_od::OrientedBox;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

Marker boxMarker(const OrientedBox& b, const std_msgs::msg::Header& header, const std::string& ns, int id,
                 float r, float g, float bl, float a) {
    Marker m;
    m.header = header;
    m.ns = ns;
    m.id = id;
    m.type = Marker::CUBE;
    m.action = Marker::ADD;
    m.pose.position.x = b.center.x();
    m.pose.position.y = b.center.y();
    m.pose.position.z = b.center.z();
    m.pose.orientation.z = std::sin(0.5 * b.yaw);
    m.pose.orientation.w = std::cos(0.5 * b.yaw);
    m.scale.x = std::max(0.05f, b.size.x());
    m.scale.y = std::max(0.05f, b.size.y());
    m.scale.z = std::max(0.05f, b.size.z());
    m.color.r = r;
    m.color.g = g;
    m.color.b = bl;
    m.color.a = a;
    m.lifetime = rclcpp::Duration::from_seconds(0.25);
    return m;
}

class LidarOdNode : public rclcpp::Node {
public:
    LidarOdNode() : Node("lidar_od") {
        const std::string path = declare_parameter<std::string>("config_file", "");
        const lidar_od::Config config = path.empty() ? lidar_od::Config{} : lidar_od::loadConfig(path);
        pipeline_ = std::make_unique<lidar_od::Pipeline>(config);

        ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/ground", 1);
        obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacles", 1);
        detection_pub_ = create_publisher<MarkerArray>("~/detections", 1);
        track_pub_ = create_publisher<MarkerArray>("~/tracks", 1);
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });
        RCLCPP_INFO(get_logger(), "lidar_od ready (config: %s)", path.empty() ? "defaults" : path.c_str());
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2& msg) {
        lidar_od::CloudPtr cloud(new lidar_od::Cloud);
        pcl::fromROSMsg(msg, *cloud);
        const double stamp = rclcpp::Time(msg.header.stamp).seconds();
        if (last_stamp_ > 0.0 && stamp < last_stamp_) {
            RCLCPP_WARN(get_logger(), "time went backwards (bag loop?), resetting tracker");
            pipeline_->reset();
        }
        last_stamp_ = stamp;

        const lidar_od::FrameResult r = pipeline_->process(cloud, stamp);

        publishSubset(*cloud, r.ground, msg.header, *ground_pub_);
        publishSubset(*cloud, r.nonground, msg.header, *obstacle_pub_);

        MarkerArray detections;
        for (const auto& d : r.detections)
            detections.markers.push_back(boxMarker(d.box, msg.header, "detections", d.id, 0.9f, 0.9f, 0.9f, 0.35f));
        detection_pub_->publish(detections);

        MarkerArray tracks;
        for (const auto& t : r.tracks) {
            if (!t.confirmed) continue;
            const float speed = t.velocity.norm();
            const bool moving = speed > 1.0f;
            tracks.markers.push_back(boxMarker(t.box_sensor, msg.header, "tracks", t.id, moving ? 1.0f : 0.4f,
                                               moving ? 0.45f : 0.6f, moving ? 0.1f : 0.9f, 0.6f));
            Marker text;
            text.header = msg.header;
            text.ns = "track_labels";
            text.id = t.id;
            text.type = Marker::TEXT_VIEW_FACING;
            text.action = Marker::ADD;
            text.pose.position.x = t.box_sensor.center.x();
            text.pose.position.y = t.box_sensor.center.y();
            text.pose.position.z = t.box_sensor.center.z() + 0.5 * t.box_sensor.size.z() + 0.5;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.6;
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
            std::ostringstream label;
            label << '#' << t.id << ' ' << std::fixed << std::setprecision(1) << speed * 3.6f << " km/h";
            text.text = label.str();
            text.lifetime = rclcpp::Duration::from_seconds(0.25);
            tracks.markers.push_back(text);
        }
        track_pub_->publish(tracks);

        RCLCPP_DEBUG(get_logger(), "%zu detections, %zu tracks, %.1f ms", r.detections.size(), r.tracks.size(),
                     r.timing.total);
    }

    static void publishSubset(const lidar_od::Cloud& cloud, const lidar_od::Indices& idx,
                              const std_msgs::msg::Header& header,
                              rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub) {
        if (pub.get_subscription_count() == 0) return;
        lidar_od::Cloud subset;
        subset.reserve(idx.size());
        for (int i : idx) subset.push_back(cloud[i]);
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(subset, out);
        out.header = header;
        pub.publish(out);
    }

    std::unique_ptr<lidar_od::Pipeline> pipeline_;
    double last_stamp_ = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_, obstacle_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr detection_pub_, track_pub_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdNode>());
    rclcpp::shutdown();
    return 0;
}
