# LiDAR 3D Object Detection

<img src="media/final.gif" width="700" height="400" />

## Overview

This project implements a complete LiDAR-based 3D object detection and tracking pipeline for autonomous vehicles. The system processes point cloud data from LiDAR sensors to detect, classify, and track obstacles in real-time.

## Key Features

### Custom Algorithm Implementations
- **3D RANSAC Plane Segmentation** - Custom implementation for ground plane removal
- **3D KD-Tree** - Efficient spatial data structure for point cloud operations
- **3D Euclidean Clustering** - Custom clustering algorithm for object grouping

### Point Cloud Processing Pipeline
1. **Voxel Grid Downsampling** - Reduces computational load while preserving structure
2. **Region of Interest Filtering** - Focuses on relevant areas around the vehicle
3. **Ground Plane Segmentation** - Separates road surface from obstacles using custom RANSAC
4. **Euclidean Clustering** - Groups obstacle points using custom KD-Tree
5. **Bounding Box Generation** - Creates oriented bounding boxes using PCA

### Technical Specifications
- **Environment**: Ubuntu 20.04
- **PCL Version**: 1.11
- **C++ Standard**: C++14
- **Compiler**: GCC 9.4.0


## Project Structure

```
LIDAR_3D_OD/
├── src/
│   ├── environment.cpp          # Main application and visualization
│   ├── processPointClouds.cpp   # Point cloud processing with custom algorithms
│   ├── processPointClouds.h     # Processing class with RANSAC and clustering
│   ├── render/                  # Visualization utilities
│   ├── sensors/                 # LiDAR sensor simulation
│   └── algorithms/
│       ├── cluster/
│       │   └── kdtree.h        # Custom 3D KD-Tree implementation
│       └── ransac/
│           └── ransac.cpp      # RANSAC plane segmentation
└── build/                      # Build artifacts
```

## Installation

### Prerequisites

- Ubuntu 20.04 (or compatible Linux distribution)
- PCL 1.11 or higher
- Eigen 3
- C++14 compatible compiler

### Build Instructions

1. **Clone the repository**
   ```bash
   git clone https://github.com/22by7-raikar/LIDAR_3D_OD.git
   cd LIDAR_3D_OD
   ```

2. **Install dependencies**
   ```bash
   sudo apt install libpcl-dev libeigen3-dev
   ```

3. **Build the project**
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

4. **Run the application**
   ```bash
   ./environment
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

### Plane Segmentation
- Max iterations: 25
- Distance threshold: 0.3m

### Clustering
- Cluster tolerance: 0.53m
- Min cluster size: 10 points
- Max cluster size: 500 points

## Results

The system successfully:
- ✅ Detects and segments ground plane using custom RANSAC
- ✅ Clusters obstacle points using custom KD-Tree and Euclidean clustering
- ✅ Generates oriented bounding boxes around detected objects
- ✅ Processes real-world LiDAR data streams in real-time

## Future Improvements

- Add object classification (car, pedestrian, cyclist)
- Implement multi-frame object tracking
- Optimize KD-Tree for better performance
- Add velocity estimation for detected objects

## License

This project is part of the Udacity Sensor Fusion Nanodegree program.

## Acknowledgments

- Udacity Sensor Fusion Nanodegree
- Point Cloud Library (PCL) community
