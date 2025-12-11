// Enhanced environment with object tracking for bicyclist detection challenge
// Extends base environment.cpp with multi-object tracking capabilities

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "Tracker.h"
#include "processPointClouds.cpp"
#include <iomanip>

// Color mapping for different object types
Color getColorForLabel(const std::string& label) {
    if (label == "bicycle") return Color(1, 0.5, 0);      // Orange
    if (label == "pedestrian") return Color(1, 1, 0);     // Yellow
    if (label == "car") return Color(0, 1, 1);             // Cyan
    if (label == "truck") return Color(0.5, 0, 1);         // Purple
    return Color(1, 1, 1);                                  // White
}

void cityBlockWithTracking(pcl::visualization::PCLVisualizer::Ptr& viewer, 
                           ProcessPointClouds<pcl::PointXYZI>& pointProcessor,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                           ObjectTracker& tracker,
                           int& frameCount) {
    
    // Filter point cloud
    inputCloud = pointProcessor.FilterCloud(inputCloud, 0.3f, 
                                            Eigen::Vector4f(-10, -5, -2, 1), 
                                            Eigen::Vector4f(30, 8, 1, 1));

    // Segment plane
    auto segmentCloud = pointProcessor.SegmentPlane(inputCloud, 25, 0.3f);

    // Cluster obstacles
    auto cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.53f, 10, 500);

    // Generate bounding boxes
    std::vector<BoxQ> boxes;
    for (const auto& cluster : cloudClusters) {
        BoxQ box = pointProcessor.PCABoundingBox(cluster);
        boxes.push_back(box);
    }

    // Update tracker
    auto trackedObjects = tracker.update(boxes);

    // Render point clouds
    renderPointCloud(viewer, segmentCloud.second, "road", Color(0, 1, 0));

    // Render tracked objects with IDs and labels
    for (const auto& pair : trackedObjects) {
        const TrackedObject& obj = pair.second;
        
        // Skip objects with low confidence
        if (obj.confidence < 0.3f) continue;
        
        Color objColor = getColorForLabel(obj.label);
        
        // Render bounding box
        renderBox(viewer, obj.box, obj.id, objColor);
        
        // Add text label with ID, class, and velocity
        std::stringstream ss;
        ss << "ID:" << obj.id << " " << obj.label << "\n"
           << std::fixed << std::setprecision(1)
           << "v=" << obj.velocity.norm() << "m/s";
        
        pcl::PointXYZ textPos;
        textPos.x = obj.position(0);
        textPos.y = obj.position(1);
        textPos.z = obj.position(2) + obj.dimensions(2) * 0.5f + 0.5f;  // Above box
        
        viewer->addText3D(ss.str(), textPos, 0.5, 
                         objColor.r, objColor.g, objColor.b,
                         "label_" + std::to_string(obj.id));
    }

    // Display tracking statistics
    auto vrus = tracker.getVulnerableRoadUsers();
    
    std::cout << "\n=== Frame " << frameCount << " ===\n";
    std::cout << "Total tracked objects: " << trackedObjects.size() << "\n";
    std::cout << "Vulnerable road users: " << vrus.size() << "\n";
    
    for (const auto& vru : vrus) {
        std::cout << "  - ID " << vru.id << " (" << vru.label << "): "
                  << "pos=(" << std::fixed << std::setprecision(1)
                  << vru.position(0) << ", " << vru.position(1) << ", " << vru.position(2) << ") "
                  << "vel=" << vru.velocity.norm() << " m/s "
                  << "age=" << vru.age << " frames\n";
    }
    
    frameCount++;
}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    int distance = 16;
    
    switch(setAngle) {
        case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
    }

    if(setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char** argv) {
    std::cout << "=== LiDAR 3D Object Detection with Tracking ===\n";
    std::cout << "Challenge: Detect and track bicyclist and surrounding obstacles\n\n";

    pcl::visualization::PCLVisualizer::Ptr viewer = 
        std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer with Tracking");
    
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    std::vector<boost::filesystem::path> stream = 
        pointProcessor.streamPcd("../src/sensors/data/pcd/data_2");
    
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // Initialize tracker (2.0m max distance, 5 max missed frames, 0.1s between frames)
    ObjectTracker tracker(2.0f, 5, 0.1f);
    int frameCount = 0;

    std::cout << "Controls:\n";
    std::cout << "  - Space: Pause/Resume\n";
    std::cout << "  - R: Reset tracking\n";
    std::cout << "  - Q: Quit\n\n";

    while (!viewer->wasStopped()) {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessor.loadPcd((*streamIterator).string());
        cityBlockWithTracking(viewer, pointProcessor, inputCloudI, tracker, frameCount);
            
        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
            tracker.reset();  // Reset tracking on loop
            frameCount = 0;
            std::cout << "\n=== Looping back to start ===\n";
        }

        viewer->spinOnce(100);  // 100ms delay for visualization
    }

    return 0;
}
