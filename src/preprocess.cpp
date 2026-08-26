#include "preprocess.hpp"

#include <cmath>
#include <pcl/filters/voxel_grid.h>

CloudI::Ptr cropByRange(const CloudI::Ptr& cloud, float minRange, float maxRange) {
    CloudI::Ptr out(new CloudI);
    for (size_t i = 0; i < cloud->size(); ++i) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float d = std::sqrt(x * x + y * y);
        if (d > minRange && d < maxRange) {
            out->push_back(cloud->points[i]);
        }
    }
    return out;
}

CloudI::Ptr downsample(const CloudI::Ptr& cloud, float leaf) {
    CloudI::Ptr out(new CloudI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
    return out;
}
