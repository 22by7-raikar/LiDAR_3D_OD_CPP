/// @file
/// @brief Multi-object tracker.
#pragma once

#include "lidar_od/config.h"
#include "lidar_od/tracking/kalman_filter.h"
#include "lidar_od/types.h"

#include <deque>
#include <vector>

namespace lidar_od {

/// @brief Life-cycle state of a track. Deleted tracks are removed from the tracker.
enum class TrackState {
    Tentative,  ///< New; not yet seen often enough to be trusted.
    Confirmed,  ///< Seen in at least TrackerParams::confirm_hits frames.
};

/// @brief One tracked object.
struct Track {
    int id = -1;                                ///< Identity, unique within a tracker.
    TrackState state = TrackState::Tentative;   ///< Life-cycle state.
    ConstantVelocityKalman kf;                  ///< Position and velocity filter.
    Eigen::Vector3f size = Eigen::Vector3f::Zero();  ///< Smoothed box size [m].
    float yaw = 0.0f;                           ///< Smoothed box heading [rad].
    float z = 0.0f;                             ///< Smoothed box centre height [m].
    int hits = 1;                               ///< Frames with an associated detection.
    int misses = 0;                             ///< Consecutive frames without one.
    int age = 1;                                ///< Frames since creation.
    int detection = -1;                         ///< Detection matched this frame, -1 when coasting.
    std::deque<Eigen::Vector2f> history;        ///< Recent positions, oldest first.

    /// @brief Starts a track at a detection.
    /// @param kp Kalman noise model.
    /// @param box First detection.
    /// @param track_id Identity to use.
    Track(const KalmanParams& kp, const OrientedBox& box, int track_id);

    /// @brief Current box estimate.
    OrientedBox box() const;
};

/// @brief Tracks boxes over time in a fixed (world) frame.
///
/// - Constant-velocity Kalman filter per track.
/// - Gating: squared Mahalanobis distance below TrackerParams::gate_chi2 and
///   Euclidean distance below TrackerParams::max_distance.
/// - Cost: negative log-likelihood d² + ln|S|, so tracks with a wide gate do
///   not win by default.
/// - Matching cascade: confirmed tracks are assigned first; tentative tracks
///   only compete for the detections left over. Each stage is solved
///   optimally with solveAssignment().
/// - Life cycle: tentative → confirmed after TrackerParams::confirm_hits hits;
///   deleted after too many consecutive misses.
///
/// Box size, height and yaw are exponentially smoothed outside the filter.
class MultiObjectTracker {
public:
    /// @param params Tracker parameters.
    explicit MultiObjectTracker(const TrackerParams& params);

    /// @brief Predicts all tracks to @p timestamp and associates the detections.
    /// @param detections Boxes in the world frame.
    /// @param timestamp Frame time [s].
    void update(const std::vector<OrientedBox>& detections, double timestamp);

    /// @brief Deletes all tracks and restarts the ids at 0.
    void reset();

    /// @brief Live tracks after the last update.
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
