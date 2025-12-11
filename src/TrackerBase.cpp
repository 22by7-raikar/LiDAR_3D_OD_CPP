#include "TrackerBase.h"
#include <algorithm>

TrackerBase::TrackerBase(float maxDistance, int maxMissed, float deltaTime)
    : nextId_(0), maxAssociationDistance_(maxDistance), 
      maxMissedFrames_(maxMissed), dt_(deltaTime) {}

std::string TrackerBase::classifyObject(const Eigen::Vector3f& dimensions) const {
    float length = dimensions(0);
    float width = dimensions(1);
    float height = dimensions(2);
    float volume = length * width * height;
    
    if (volume > 0.5f && volume < 2.5f && width < 1.0f && height > 1.2f && height < 2.0f) {
        return "bicycle";
    }
    if (volume > 0.3f && volume < 1.5f && height > 1.4f && height < 2.2f) {
        return "pedestrian";
    }
    if (volume > 4.0f && volume < 25.0f && height > 1.2f && height < 2.5f) {
        return "car";
    }
    if (volume > 20.0f) {
        return "truck";
    }
    return "unknown";
}

void TrackerBase::reset() {
    trackedObjects_.clear();
    nextId_ = 0;
}

std::vector<TrackedObject> TrackerBase::getVulnerableRoadUsers() const {
    std::vector<TrackedObject> vrus;
    for (const auto& pair : trackedObjects_) {
        const TrackedObject& obj = pair.second;
        if (obj.label == "bicycle" || obj.label == "pedestrian") {
            vrus.push_back(obj);
        }
    }
    return vrus;
}
