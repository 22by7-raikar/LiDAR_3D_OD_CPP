# LiDAR 3D Object Detection

<img src="media/final.gif" width="700" height="400" />

## Overview

C++14 implementation of LiDAR-based 3D object detection with custom RANSAC, KD-Tree, and clustering algorithms.

## Features

- **Custom 3D RANSAC** - Ground plane segmentation
- **Custom 3D KD-Tree** - Spatial indexing for fast nearest neighbor search  
- **Custom Euclidean Clustering** - Object grouping with KD-Tree optimization
- **Object Tracking** - Multi-object tracking with velocity estimation and classification

## Requirements

- Ubuntu 20.04+
- PCL 1.11+
- Eigen 3
- CMake 3.10+
- C++14 compiler

## Build & Run

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)

# Basic detection
./environment

# With tracking (bicyclist challenge)
./environment_tracking
```

## Project Structure

```
├── include/              # Headers
│   ├── Tracker.h
│   ├── processPointClouds.h
│   └── algorithms/cluster/kdtree.h
├── src/                  # Implementation
│   ├── environment.cpp              # Basic detection
│   ├── environment_tracking.cpp     # With tracking
│   ├── processPointClouds.cpp
│   └── Tracker.cpp
└── build/
    ├── environment           # Basic executable
    └── environment_tracking  # Tracking executable
```

## Algorithm Details

### RANSAC 3D Plane Segmentation
- Randomly samples 3 points to define a plane
- Computes plane equation using cross product for normal vector
- Iteratively finds plane with maximum inliers
- Used for ground plane removal

### KD-Tree 3D
- Balanced binary tree for 3D spatial indexing
- Splits alternately on x, y, z dimensions
- Efficient nearest neighbor search (O(log n) average case)
- Used for clustering and spatial queries

### Euclidean Clustering
- Proximity-based grouping using custom KD-Tree
- Recursive region growing algorithm
- Configurable distance tolerance and cluster size limits
- Separates individual objects from point cloud

## Configuration Parameters

### Point Cloud Filtering
- Voxel size: 0.3m
- ROI: X [-10, 30], Y [-5, 8], Z [-2, 1]


## Algorithm Details

### RANSAC (3D Plane Segmentation)
- Iterations: 25
- Distance threshold: 0.3m
- Randomly samples 3 points, fits plane via cross product

### KD-Tree (3D Spatial Indexing)
- Balanced binary tree with alternating split dimensions (x, y, z)
- O(log n) search complexity
- Used for efficient nearest neighbor queries

### Euclidean Clustering
- Tolerance: 0.53m
- Min/max size: 10-500 points
- Recursive region growing using KD-Tree

## Tracking Features

The `environment_tracking` executable adds:
- Persistent object IDs across frames
- Velocity estimation (m/s)
- Object classification (bicycle, pedestrian, car, truck)
- Color-coded visualization by object type

### Tracker Architecture (Extensible)

The tracking system uses an **extensible architecture** for easy algorithm upgrades:

- **`TrackerBase`** - Abstract base class with common functionality
- **`SimpleTracker`** - Current implementation (nearest neighbor + constant velocity)
- **`KalmanTracker`** - Stub for Kalman filter + Hungarian algorithm (TODO)

To switch algorithms, simply change the tracker type in `environment_tracking.cpp`:
```cpp
// Current
SimpleTracker tracker(2.0f, 5, 0.1f);

// Future (after implementation)
KalmanTracker tracker(2.0f, 5, 0.1f);
```

See `MERGE_GUIDE.md` for implementation details.

## License

Udacity Sensor Fusion Nanodegree project.
