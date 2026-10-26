#pragma once

#include "lidar_od/config.h"
#include "lidar_od/tracking/kalman_filter.h"
#include "lidar_od/types.h"

#include <deque>
#include <vector>

namespace lidar_od {

enum class TrackState { Tentative, Confirmed };

struct Track {
    int id = -1;
    TrackState state = TrackState::Tentative;
    ConstantVelocityKalman kf;
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    float yaw = 0.0f;
    float z = 0.0f;   // box centre height
    int hits = 1;     // frames with an associated detection
    int misses = 0;   // consecutive frames without one
    int age = 1;      // frames since birth
    int detection = -1;  // index of the detection matched this frame, -1 if coasting
    std::deque<Eigen::Vector2f> history;

    Track(const KalmanParams& kp, const OrientedBox& box, int track_id);
    OrientedBox box() const;
};

// Multi-object tracker in the world frame:
//  - constant-velocity Kalman filter per track,
//  - Mahalanobis gating plus a hard distance gate,
//  - matching cascade (confirmed tracks first), each stage solved optimally
//    with the Hungarian algorithm on a negative log-likelihood cost,
//  - track life cycle tentative -> confirmed -> deleted.
class MultiObjectTracker {
public:
    explicit MultiObjectTracker(const TrackerParams& params);

    // `detections` are boxes in the world frame, `timestamp` in seconds.
    void update(const std::vector<OrientedBox>& detections, double timestamp);
    void reset();

    const std::vector<Track>& tracks() const { return tracks_; }

private:
    void associate(const std::vector<OrientedBox>& detections, TrackState state,
                   std::vector<bool>& detection_used);

    TrackerParams params_;
    std::vector<Track> tracks_;
    int next_id_ = 0;
    double last_time_ = 0.0;
    bool initialised_ = false;
};

}  // namespace lidar_od
