#ifndef TRACKER_BASE_H_
#define TRACKER_BASE_H_

#include <vector>
#include <map>
#include <Eigen/Core>
#include "render/box.h"

/**
 * @brief Tracked object state
 */
struct TrackedObject {
    int id;
    BoxQ box;
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f dimensions;
    int age;
    int missedFrames;
    float confidence;
    std::string label;
    
    // For Kalman filter (future)
    Eigen::VectorXf state;           // State vector (can extend to 6D or more)
    Eigen::MatrixXf covariance;      // State covariance
    
    TrackedObject() : id(-1), age(0), missedFrames(0), confidence(1.0f), label("unknown") {
        position = Eigen::Vector3f::Zero();
        velocity = Eigen::Vector3f::Zero();
        dimensions = Eigen::Vector3f::Zero();
    }
};

/**
 * @brief Base class for object tracking - extensible for advanced algorithms
 */
class TrackerBase {
protected:
    std::map<int, TrackedObject> trackedObjects_;
    int nextId_;
    float maxAssociationDistance_;
    int maxMissedFrames_;
    float dt_;
    
    virtual std::string classifyObject(const Eigen::Vector3f& dimensions) const;
    
public:
    TrackerBase(float maxDistance = 2.0f, int maxMissed = 5, float deltaTime = 0.1f);
    virtual ~TrackerBase() = default;
    
    /**
     * @brief Update tracker with new detections - override for different algorithms
     */
    virtual std::map<int, TrackedObject> update(const std::vector<BoxQ>& boxes) = 0;
    
    const std::map<int, TrackedObject>& getTrackedObjects() const { return trackedObjects_; }
    void reset();
    std::vector<TrackedObject> getVulnerableRoadUsers() const;
};

#endif /* TRACKER_BASE_H_ */
