#ifndef KALMAN_TRACKER_H_
#define KALMAN_TRACKER_H_

#include "TrackerBase.h"

/**
 * @brief Kalman filter tracker with Hungarian algorithm
 * TODO: Implement for improved tracking accuracy and multi-object association
 * 
 * Features to add:
 * - 6D state vector: [x, y, z, vx, vy, vz]
 * - Prediction step: x_pred = F * x + w (process noise)
 * - Update step: x_new = x_pred + K * (z - H * x_pred)
 * - Hungarian algorithm for optimal assignment
 * - Mahalanobis distance for gating
 */
class KalmanTracker : public TrackerBase {
private:
    // Kalman filter matrices
    Eigen::MatrixXf F_;  // State transition matrix
    Eigen::MatrixXf H_;  // Observation matrix
    Eigen::MatrixXf Q_;  // Process noise covariance
    Eigen::MatrixXf R_;  // Measurement noise covariance
    
    void initKalmanMatrices();
    void predictTrack(TrackedObject& obj);
    void updateTrack(TrackedObject& obj, const Eigen::Vector3f& measurement);
    
    // Hungarian algorithm for optimal assignment (TODO)
    std::vector<std::pair<int, int>> hungarianAssignment(
        const std::vector<Eigen::Vector3f>& detections,
        const std::map<int, TrackedObject>& tracks);
    
public:
    KalmanTracker(float maxDistance = 2.0f, int maxMissed = 5, float deltaTime = 0.1f);
    
    std::map<int, TrackedObject> update(const std::vector<BoxQ>& boxes) override;
};

#endif /* KALMAN_TRACKER_H_ */
