#ifndef SIMPLE_TRACKER_H_
#define SIMPLE_TRACKER_H_

#include "TrackerBase.h"

/**
 * @brief Simple nearest-neighbor tracker with constant velocity model
 * Current implementation - baseline for comparison
 */
class SimpleTracker : public TrackerBase {
private:
    int findClosestMatch(const Eigen::Vector3f& position, float& distance);
    
public:
    SimpleTracker(float maxDistance = 2.0f, int maxMissed = 5, float deltaTime = 0.1f);
    
    std::map<int, TrackedObject> update(const std::vector<BoxQ>& boxes) override;
};

#endif /* SIMPLE_TRACKER_H_ */
