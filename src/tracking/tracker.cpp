#include "lidar_od/tracking/tracker.h"
#include "lidar_od/tracking/hungarian.h"

#include <algorithm>
#include <cmath>

namespace lidar_od {
namespace {
constexpr double kForbidden = 1e9;
}

Track::Track(const KalmanParams& kp, const OrientedBox& b, int track_id)
    : id(track_id), kf(kp, b.center.head<2>()), size(b.size), yaw(b.yaw), z(b.center.z()) {
    history.push_back(b.center.head<2>());
}

OrientedBox Track::box() const {
    OrientedBox b;
    b.center << kf.position(), z;
    b.size = size;
    b.yaw = yaw;
    return b;
}

MultiObjectTracker::MultiObjectTracker(const TrackerParams& params) : params_(params) {}

void MultiObjectTracker::reset() {
    tracks_.clear();
    next_id_ = 0;
    initialised_ = false;
}

void MultiObjectTracker::associate(const std::vector<OrientedBox>& detections, TrackState state,
                                   std::vector<bool>& detection_used) {
    std::vector<int> rows, cols;
    for (size_t i = 0; i < tracks_.size(); ++i)
        if (tracks_[i].state == state && tracks_[i].detection < 0) rows.push_back(static_cast<int>(i));
    for (size_t j = 0; j < detections.size(); ++j)
        if (!detection_used[j]) cols.push_back(static_cast<int>(j));
    if (rows.empty() || cols.empty()) return;

    // Cost = negative log-likelihood of the detection under the track's
    // predicted position: d^2 (squared Mahalanobis distance) + ln|S|. The ln|S|
    // term stops uncertain tracks from winning just because their gate is wide.
    // Pairs outside the chi-square gate or the hard distance gate are forbidden.
    std::vector<std::vector<double>> cost(rows.size(), std::vector<double>(cols.size(), kForbidden));
    for (size_t r = 0; r < rows.size(); ++r) {
        const Track& t = tracks_[rows[r]];
        const double log_det = std::log(t.kf.innovationCovariance().determinant());
        for (size_t c = 0; c < cols.size(); ++c) {
            const Eigen::Vector2f z = detections[cols[c]].center.head<2>();
            const float m2 = t.kf.mahalanobis2(z);
            if (m2 <= params_.gate_chi2 && (z - t.kf.position()).norm() <= params_.max_distance)
                cost[r][c] = m2 + log_det;
        }
    }

    const std::vector<int> assignment = solveAssignment(cost);
    const float a = params_.size_smoothing;
    for (size_t r = 0; r < rows.size(); ++r) {
        const int c = assignment[r];
        if (c < 0 || cost[r][c] >= kForbidden) continue;
        Track& t = tracks_[rows[r]];
        const int j = cols[c];
        const OrientedBox& d = detections[j];
        t.kf.update(d.center.head<2>());
        // Shape is not part of the filter: smooth it, respecting that a box
        // heading is only defined modulo 180 degrees.
        t.size = (1.0f - a) * t.size + a * d.size;
        t.yaw = wrapHalfPi(t.yaw + a * wrapHalfPi(d.yaw - t.yaw));
        t.z = (1.0f - a) * t.z + a * d.center.z();
        ++t.hits;
        t.detection = j;
        detection_used[j] = true;
    }
}

void MultiObjectTracker::update(const std::vector<OrientedBox>& detections, double timestamp) {
    const float dt = initialised_ ? static_cast<float>(timestamp - last_time_) : 0.0f;
    last_time_ = timestamp;
    initialised_ = true;

    for (Track& t : tracks_) {
        t.kf.predict(dt);
        t.detection = -1;
        ++t.age;
    }

    // Matching cascade: confirmed tracks pick first, tentative tracks only get
    // what is left. Otherwise a fresh track, whose large covariance makes any
    // nearby detection look plausible, can steal the detection of an
    // established object and cause an identity switch.
    std::vector<bool> detection_used(detections.size(), false);
    associate(detections, TrackState::Confirmed, detection_used);
    associate(detections, TrackState::Tentative, detection_used);
    for (Track& t : tracks_) {
        if (t.detection < 0) {
            ++t.misses;
        } else {
            t.misses = 0;
            if (t.state == TrackState::Tentative && t.hits >= params_.confirm_hits)
                t.state = TrackState::Confirmed;
        }
        t.history.push_back(t.kf.position());
        while (static_cast<int>(t.history.size()) > params_.history_length) t.history.pop_front();
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track& t) {
                                     const int limit = t.state == TrackState::Confirmed
                                                           ? params_.max_missed_confirmed
                                                           : params_.max_missed_tentative;
                                     return t.misses > limit;
                                 }),
                  tracks_.end());

    for (size_t j = 0; j < detections.size(); ++j)
        if (!detection_used[j]) {
            tracks_.emplace_back(params_.kalman, detections[j], next_id_++);
            tracks_.back().detection = static_cast<int>(j);
        }
}

}  // namespace lidar_od
