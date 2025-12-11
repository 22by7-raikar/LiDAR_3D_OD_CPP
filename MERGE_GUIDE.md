# Merging and Future Development Guide

## Current Repository Structure

This codebase is now structured for easy extension and merging:

### Repository to Merge To
Target: `https://github.com/22by7-raikar/Classical-3D_LiDAR_Detection`

### Current Repository
Source: `https://github.com/22by7-raikar/LiDAR_3D_OD_CPP` (this repo)

---

## Pre-Merge Checklist

Before merging to Classical-3D_LiDAR_Detection:

```bash
# 1. Stage all changes
git add -A

# 2. Commit with descriptive message
git commit -m "Refactor: Extensible tracker architecture with Simple/Kalman options

- Split tracker into base class (TrackerBase) and implementations
- SimpleTracker: Current working implementation (nearest neighbor + const velocity)
- KalmanTracker: Stub for future Kalman filter + Hungarian algorithm
- Backward compatible via Tracker.h alias
- Clean codebase: removed excess docs, organized headers
"

# 3. Push to current repo
git push origin master

# 4. Add target repo as remote
git remote add target https://github.com/22by7-raikar/Classical-3D_LiDAR_Detection.git
git fetch target

# 5. Merge (or create PR)
# Option A: Direct merge
git checkout -b merge-improvements
git merge target/master --allow-unrelated-histories
# Resolve conflicts if any

# Option B: Create Pull Request on GitHub
# - Fork Classical-3D_LiDAR_Detection
# - Push this branch
# - Create PR with description
```

---

## Extensible Architecture

### Current Structure

```
include/
├── TrackerBase.h          # Abstract base class
├── SimpleTracker.h        # Working implementation (current)
├── KalmanTracker.h        # Stub for future Kalman + Hungarian
└── Tracker.h              # Compatibility alias

src/
├── TrackerBase.cpp        # Common functionality (classification, etc.)
├── SimpleTracker.cpp      # Nearest neighbor + constant velocity
├── KalmanTracker.cpp      # TODO: Full implementation
└── environment_tracking.cpp
```

### How to Switch Trackers

In `environment_tracking.cpp`, change:
```cpp
// Current (Simple)
SimpleTracker tracker(2.0f, 5, 0.1f);

// Future (Kalman + Hungarian)
KalmanTracker tracker(2.0f, 5, 0.1f);
```

---

## Adding Hungarian Algorithm

### Steps:

1. **Install Hungarian library** (optional - can implement from scratch):
```bash
# Option A: Use existing library
git clone https://github.com/mcximing/hungarian-algorithm-cpp
# Add to project

# Option B: Implement yourself (recommended for learning)
```

2. **Implement in `KalmanTracker.cpp`**:
```cpp
std::vector<std::pair<int, int>> KalmanTracker::hungarianAssignment(
    const std::vector<Eigen::Vector3f>& detections,
    const std::map<int, TrackedObject>& tracks) {
    
    // Build cost matrix
    int n_det = detections.size();
    int n_track = tracks.size();
    Eigen::MatrixXf costMatrix(n_det, n_track);
    
    int j = 0;
    for (const auto& pair : tracks) {
        for (int i = 0; i < n_det; i++) {
            // Use Mahalanobis distance instead of Euclidean
            costMatrix(i, j) = mahalanobisDistance(
                detections[i], 
                pair.second.position,
                pair.second.covariance
            );
        }
        j++;
    }
    
    // Apply Hungarian algorithm
    std::vector<int> assignment = hungarianAlgorithm(costMatrix);
    
    // Convert to pairs and apply gating threshold
    std::vector<std::pair<int, int>> matches;
    for (int i = 0; i < n_det; i++) {
        if (assignment[i] >= 0 && costMatrix(i, assignment[i]) < gatingThreshold_) {
            matches.emplace_back(i, assignment[i]);
        }
    }
    
    return matches;
}
```

---

## Adding Kalman Filter

### Steps:

1. **Complete the prediction step** in `KalmanTracker::predictTrack()`:
```cpp
void KalmanTracker::predictTrack(TrackedObject& obj) {
    // State: [x, y, z, vx, vy, vz]
    obj.state = F_ * obj.state;
    obj.covariance = F_ * obj.covariance * F_.transpose() + Q_;
    
    // Extract position for visualization
    obj.position = obj.state.head<3>();
}
```

2. **Complete the update step** in `KalmanTracker::updateTrack()`:
```cpp
void KalmanTracker::updateTrack(TrackedObject& obj, const Eigen::Vector3f& measurement) {
    // Innovation
    Eigen::Vector3f y = measurement - H_ * obj.state;
    
    // Innovation covariance
    Eigen::Matrix3f S = H_ * obj.covariance * H_.transpose() + R_;
    
    // Kalman gain
    Eigen::MatrixXf K = obj.covariance * H_.transpose() * S.inverse();
    
    // Update state and covariance
    obj.state = obj.state + K * y;
    obj.covariance = (Eigen::MatrixXf::Identity(6, 6) - K * H_) * obj.covariance;
    
    // Extract for visualization
    obj.position = obj.state.head<3>();
    obj.velocity = obj.state.tail<3>();
}
```

3. **Update the main loop** in `KalmanTracker::update()`:
```cpp
std::map<int, TrackedObject> KalmanTracker::update(const std::vector<BoxQ>& boxes) {
    // 1. Predict all tracks
    for (auto& pair : trackedObjects_) {
        predictTrack(pair.second);
    }
    
    // 2. Extract measurements
    std::vector<Eigen::Vector3f> measurements = extractPositions(boxes);
    
    // 3. Hungarian assignment
    auto matches = hungarianAssignment(measurements, trackedObjects_);
    
    // 4. Update matched tracks
    for (const auto& match : matches) {
        updateTrack(trackedObjects_[match.second], measurements[match.first]);
        // ... update other fields
    }
    
    // 5. Create new tracks for unmatched detections
    // 6. Delete old tracks
    
    return trackedObjects_;
}
```

---

## Testing New Implementations

```bash
# Build
cd build
make -j$(nproc)

# Test Simple tracker (current - works)
./environment_tracking

# Test Kalman tracker (after implementation)
# Change tracker type in environment_tracking.cpp first
./environment_tracking
```

---

## Recommended Development Order

1. ✅ **Done**: Refactor to extensible architecture
2. **Next**: Implement Hungarian algorithm
3. **Then**: Complete Kalman filter
4. **Finally**: Add advanced features:
   - IMM (Interacting Multiple Model) for different motion models
   - Deep SORT with appearance features
   - EKF for non-linear motion
   - Multi-sensor fusion

---

## Resources for Implementation

### Hungarian Algorithm
- Paper: "The Hungarian Method for the Assignment Problem" (Kuhn, 1955)
- Library: https://github.com/mcximing/hungarian-algorithm-cpp

### Kalman Filter
- Book: "Probabilistic Robotics" by Thrun, Burgard, Fox (Chapter 3)
- Tutorial: https://www.kalmanfilter.net/

### SORT/Deep SORT
- Paper: "Simple Online and Realtime Tracking" (Bewley et al., 2016)
- Paper: "Simple Online and Realtime Tracking with a Deep Association Metric" (Wojke et al., 2017)

---

## Git Workflow for Merging

```bash
# Create feature branch
git checkout -b feature/extensible-tracker

# Make sure everything is committed
git status

# Add target repo as remote
git remote add classical https://github.com/22by7-raikar/Classical-3D_LiDAR_Detection.git
git fetch classical

# Option 1: Merge (if repos are related)
git merge classical/master

# Option 2: Cherry-pick specific commits
git log  # Find commit hashes
git cherry-pick <commit-hash>

# Option 3: Start fresh on target repo
cd /tmp
git clone https://github.com/22by7-raikar/Classical-3D_LiDAR_Detection.git
cd Classical-3D_LiDAR_Detection
# Copy improved files
# Create new branch
# Push and create PR
```

---

## Questions Before Merging

1. **Do both repos have the same file structure?**
   - If yes → merge/rebase
   - If no → copy improved files selectively

2. **Is Classical-3D_LiDAR_Detection your main repo?**
   - If yes → merge this into that
   - If no → keep repos separate

3. **Do you want to keep git history?**
   - If yes → use git merge/rebase
   - If no → copy files and fresh commit
