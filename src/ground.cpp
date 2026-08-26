#include "ground.hpp"

#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <Eigen/Dense>

pcl::PointIndices::Ptr segmentGround(const CloudI::Ptr& cloud, float distThreshold) {
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Constrain the plane to near-horizontal so a wall is never chosen as ground.
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(0.26f);  // ~15 deg
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(distThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffs);
    return inliers;
}

CloudI::Ptr removeGround(const CloudI::Ptr& cloud, float distThreshold) {
    pcl::PointIndices::Ptr inliers = segmentGround(cloud, distThreshold);

    CloudI::Ptr out(new CloudI);
    if (inliers->indices.empty()) {
        std::cerr << "no ground plane found; keeping all points\n";
        *out = *cloud;
        return out;
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*out);
    return out;
}
