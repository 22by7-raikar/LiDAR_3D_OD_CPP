// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(const typename pcl::PointCloud<PointT>::Ptr cloud) const
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel Grid Point Reduction
    
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::VoxelGrid<PointT> vg;

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    // Region Based Filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_region = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);
    
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);   

    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();

    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    const pcl::PointIndices::Ptr inliers, const typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Created one pt cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obst_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    typename pcl::PointCloud<PointT>::Ptr plane_cloud = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    
    // Extract plane (inliers)
    extract.setNegative(false);
    extract.filter(*plane_cloud);
    
    // Extract obstacles (outliers)
    extract.setNegative(true);
    extract.filter(*obst_cloud);

    std::cerr << "PointCloud representing the planar component: " << plane_cloud->points.size() << " data points." << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, plane_cloud);

    return segResult;
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// Custom RANSAC implementation for 3D plane

	const size_t cloudSize = cloud->points.size();
	// For max iterations 
	for(int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;

		// Randomly sample 3 points
		while(inliers.size() < 3)
			inliers.insert(rand() % cloudSize);

		auto itr = inliers.begin();
		PointT point1 = cloud->points[*itr];
		itr++;
		PointT point2 = cloud->points[*itr];
		itr++;
		PointT point3 = cloud->points[*itr];

		// Calculate two vectors in the plane
		float v1_x = point2.x - point1.x;
		float v1_y = point2.y - point1.y;
		float v1_z = point2.z - point1.z;

		float v2_x = point3.x - point1.x;
		float v2_y = point3.y - point1.y;
		float v2_z = point3.z - point1.z;

		// Normal vector = v1 x v2 (cross product)
		float A = v1_y * v2_z - v1_z * v2_y;
		float B = v1_z * v2_x - v1_x * v2_z;
		float C = v1_x * v2_y - v1_y * v2_x;
		float D = -(A * point1.x + B * point1.y + C * point1.z);
		
		const float denominator = sqrt(A*A + B*B + C*C);
		
		// Skip degenerate plane (collinear points)
		if(denominator < 1e-6f)
			continue;
		// Measure distance between every point and fitted plane
		for(size_t index = 0; index < cloudSize; index++)
		{
			if(inliers.count(index) > 0)
				continue;

			PointT point = cloud->points[index];
		
			float d = fabs(A*point.x + B*point.y + C*point.z + D) / denominator;
		
			// If distance is smaller than threshold count it as inlier
			if(d <= distanceThreshold)
				inliers.insert(index);
		}
		
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;				
	}

	// Return indicies of inliers from fitted plane with most inliers
	return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    // Use custom RANSAC implementation for plane segmentation
    std::unordered_set<int> inliersSet = RansacPlane(cloud, maxIterations, distanceThreshold);

    if (inliersSet.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Convert unordered_set to PointIndices
    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();
    inliers->indices.reserve(inliersSet.size());
    for(int index : inliersSet)
        inliers->indices.push_back(index);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
 
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D<PointT>* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
		{
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, size_t minSize, size_t maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Custom Euclidean clustering using custom 3D KD-Tree
    std::unique_ptr<KdTree3D<PointT>> tree = std::make_unique<KdTree3D<PointT>>();
    
    const size_t cloudSize = cloud->points.size();
    // Insert all points into KD-Tree
    for (size_t i = 0; i < cloudSize; i++) 
    	tree->insert(cloud->points[i], i); 

    std::vector<bool> processed(cloudSize, false);

    size_t i = 0;
    while(i < cloudSize)
    {
        if(processed[i])
        {
            i++;
            continue; 
        }
        
        std::vector<int> cluster;
        clusterHelper(i, cloud, cluster, processed, tree.get(), clusterTolerance);
        
        // Filter by size constraints
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster = std::make_shared<pcl::PointCloud<PointT>>();
            cloud_cluster->points.reserve(cluster.size());

            for (int idx : cluster) 
            {
                cloud_cluster->points.push_back(cloud->points[idx]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
        
        i++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr cluster) const
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::PCABoundingBox(const typename pcl::PointCloud<PointT>::Ptr cluster) const
{
    // Axis aligned bounding box (AABB) based on PCA
    
    // Compute PCA
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    
    // Ensure right-handed coordinate system
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform and quaternion
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoxQ boxq;
    boxq.bboxTransform = bboxTransform;
    boxq.bboxQuaternion = bboxQuaternion;
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, const std::string& file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(const std::string& file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud = std::make_shared<pcl::PointCloud<PointT>>();

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(const std::string& dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}