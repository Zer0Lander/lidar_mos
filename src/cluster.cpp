#include "cluster.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

std::vector<pcl::PointIndices> clusterObjects(const CloudI::Ptr& cloud, float tolerance, int minSize, int maxSize) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);
    return clusters;
}
