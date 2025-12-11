#include "SimpleTracker.h"
#include <algorithm>
#include <cmath>
#include <iostream>

SimpleTracker::SimpleTracker(float maxDistance, int maxMissed, float deltaTime)
    : TrackerBase(maxDistance, maxMissed, deltaTime) {}

int SimpleTracker::findClosestMatch(const Eigen::Vector3f& position, float& distance) {
    int closestId = -1;
    float minDist = maxAssociationDistance_;
    
    for (auto& pair : trackedObjects_) {
        TrackedObject& obj = pair.second;
        Eigen::Vector3f predictedPos = obj.position + obj.velocity * dt_;
        float dist = (predictedPos - position).norm();
        
        if (dist < minDist) {
            minDist = dist;
            closestId = obj.id;
        }
    }
    
    distance = minDist;
    return closestId;
}

std::map<int, TrackedObject> SimpleTracker::update(const std::vector<BoxQ>& boxes) {
    std::vector<bool> matched(boxes.size(), false);
    std::vector<int> matchedTrackIds;
    
    // Extract positions and dimensions
    std::vector<Eigen::Vector3f> positions;
    std::vector<Eigen::Vector3f> dimensions;
    
    for (const auto& box : boxes) {
        positions.emplace_back(box.bboxTransform(0, 3), 
                               box.bboxTransform(1, 3), 
                               box.bboxTransform(2, 3));
        dimensions.emplace_back(box.cube_length, box.cube_width, box.cube_height);
    }
    
    // Associate detections with tracks (greedy nearest neighbor)
    for (size_t i = 0; i < boxes.size(); ++i) {
        float distance;
        int matchId = findClosestMatch(positions[i], distance);
        
        if (matchId != -1) {
            TrackedObject& obj = trackedObjects_[matchId];
            
            // Update with exponential smoothing
            Eigen::Vector3f newVelocity = (positions[i] - obj.position) / dt_;
            obj.velocity = 0.7f * obj.velocity + 0.3f * newVelocity;
            obj.position = positions[i];
            obj.dimensions = dimensions[i];
            obj.box = boxes[i];
            obj.age++;
            obj.missedFrames = 0;
            obj.confidence = std::min(1.0f, obj.confidence + 0.1f);
            obj.label = classifyObject(dimensions[i]);
            
            matched[i] = true;
            matchedTrackIds.push_back(matchId);
        }
    }
    
    // Create new tracks
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (!matched[i]) {
            TrackedObject newObj;
            newObj.id = nextId_++;
            newObj.position = positions[i];
            newObj.dimensions = dimensions[i];
            newObj.box = boxes[i];
            newObj.velocity = Eigen::Vector3f::Zero();
            newObj.age = 1;
            newObj.missedFrames = 0;
            newObj.confidence = 0.5f;
            newObj.label = classifyObject(dimensions[i]);
            
            trackedObjects_[newObj.id] = newObj;
        }
    }
    
    // Handle missed detections
    std::vector<int> toRemove;
    for (auto& pair : trackedObjects_) {
        int id = pair.first;
        TrackedObject& obj = pair.second;
        
        if (std::find(matchedTrackIds.begin(), matchedTrackIds.end(), id) == matchedTrackIds.end()) {
            obj.missedFrames++;
            obj.confidence = std::max(0.0f, obj.confidence - 0.2f);
            
            if (obj.missedFrames > maxMissedFrames_) {
                toRemove.push_back(id);
            }
        }
    }
    
    for (int id : toRemove) {
        trackedObjects_.erase(id);
    }
    
    return trackedObjects_;
}
