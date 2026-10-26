#include "lidar_od/tracking/ego_motion.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <stdexcept>

namespace lidar_od {

EgoMotionEstimator::EgoMotionEstimator(const EgoMotionParams& params) : params_(params) {
    const auto& m = params_.method;
    if (m != "ndt" && m != "icp" && m != "gicp" && m != "none")
        throw std::invalid_argument("unknown ego_motion method '" + m + "'");
}

void EgoMotionEstimator::reset() {
    previous_.reset();
    pose_.setIdentity();
    last_delta_.setIdentity();
    converged_ = true;
    fitness_ = 0.0;
}

CloudPtr EgoMotionEstimator::prepare(const Cloud& cloud, const Indices& static_points) const {
    CloudPtr subset(new Cloud);
    const float r2 = params_.max_range * params_.max_range;
    for (int i : static_points) {
        const Point& p = cloud[i];
        if (p.x * p.x + p.y * p.y <= r2) subset->push_back(p);
    }
    CloudPtr out(new Cloud);
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud(subset);
    vg.setLeafSize(params_.voxel_leaf, params_.voxel_leaf, params_.voxel_leaf);
    vg.filter(*out);
    return out;
}

Eigen::Isometry3f EgoMotionEstimator::update(const Cloud& cloud, const Indices& static_points) {
    if (!enabled()) return pose_;

    CloudPtr current = prepare(cloud, static_points);
    if (!previous_) {
        previous_ = current;
        return pose_;
    }

    // Constant-velocity prior: the car rarely changes its motion much in 0.1 s.
    const Eigen::Matrix4f guess = last_delta_.matrix();
    Cloud aligned;
    Eigen::Matrix4f delta = guess;
    if (params_.method == "ndt") {
        pcl::NormalDistributionsTransform<Point, Point> ndt;
        ndt.setResolution(params_.ndt_resolution);
        ndt.setStepSize(params_.ndt_step_size);
        ndt.setTransformationEpsilon(1e-3);
        ndt.setMaximumIterations(params_.max_iterations);
        ndt.setInputSource(current);
        ndt.setInputTarget(previous_);
        ndt.align(aligned, guess);
        converged_ = ndt.hasConverged();
        fitness_ = ndt.getFitnessScore();
        if (converged_) delta = ndt.getFinalTransformation();
    } else {
        std::unique_ptr<pcl::IterativeClosestPoint<Point, Point>> icp;
        if (params_.method == "gicp")
            icp = std::make_unique<pcl::GeneralizedIterativeClosestPoint<Point, Point>>();
        else
            icp = std::make_unique<pcl::IterativeClosestPoint<Point, Point>>();
        icp->setMaxCorrespondenceDistance(params_.max_correspondence);
        icp->setMaximumIterations(params_.max_iterations);
        icp->setTransformationEpsilon(1e-6);
        icp->setInputSource(current);
        icp->setInputTarget(previous_);
        icp->align(aligned, guess);
        converged_ = icp->hasConverged();
        fitness_ = icp->getFitnessScore();
        if (converged_) delta = icp->getFinalTransformation();
    }

    // delta maps the current scan into the previous scan's frame: T_prev_cur.
    last_delta_ = Eigen::Isometry3f(delta);
    pose_ = pose_ * last_delta_;
    previous_ = current;
    return pose_;
}

}  // namespace lidar_od
