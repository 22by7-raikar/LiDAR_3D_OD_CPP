// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include <memory>
#include "render/box.h"
#include "algorithms/cluster/kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(const typename pcl::PointCloud<PointT>::Ptr cloud) const;

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(const typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(const pcl::PointIndices::Ptr inliers, const typename pcl::PointCloud<PointT>::Ptr cloud);

    // Custom RANSAC implementation for 3D plane segmentation
    std::unordered_set<int> RansacPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    // Custom Euclidean clustering with custom KD-Tree
    void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D<PointT>* tree, float distanceTol);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(const typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, size_t minSize, size_t maxSize);

    Box BoundingBox(const typename pcl::PointCloud<PointT>::Ptr cluster) const;

    BoxQ PCABoundingBox(const typename pcl::PointCloud<PointT>::Ptr cluster) const;

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, const std::string& file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(const std::string& file);

    std::vector<boost::filesystem::path> streamPcd(const std::string& dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */