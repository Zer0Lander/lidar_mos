#include "viz.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <string>

CloudRGB::Ptr colorGround(const CloudI::Ptr& cloud, const pcl::PointIndices& ground) {
    std::vector<char> isGround(cloud->size(), 0);
    for (int i : ground.indices) isGround[i] = 1;

    CloudRGB::Ptr out(new CloudRGB);
    out->reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZRGB p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        if (isGround[i]) {
            p.r = 70; p.g = 200; p.b = 90;
        } else {
            p.r = 200; p.g = 200; p.b = 200;
        }
        out->push_back(p);
    }
    return out;
}

CloudRGB::Ptr colorByClusters(const CloudI::Ptr& cloud, const std::vector<pcl::PointIndices>& clusters) {
    CloudRGB::Ptr out(new CloudRGB);
    for (size_t k = 0; k < clusters.size(); ++k) {
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;
        for (int idx : clusters[k].indices) {
            pcl::PointXYZRGB p;
            p.x = cloud->points[idx].x;
            p.y = cloud->points[idx].y;
            p.z = cloud->points[idx].z;
            p.r = r; p.g = g; p.b = b;
            out->push_back(p);
        }
    }
    return out;
}

void drawBoxes(pcl::visualization::PCLVisualizer& viewer, const CloudI::Ptr& cloud, const std::vector<pcl::PointIndices>& clusters) {
    for (size_t k = 0; k < clusters.size(); ++k) {
        int first = clusters[k].indices[0];
        float minX = cloud->points[first].x, maxX = minX;
        float minY = cloud->points[first].y, maxY = minY;
        float minZ = cloud->points[first].z, maxZ = minZ;
        for (int idx : clusters[k].indices) {
            const auto& p = cloud->points[idx];
            minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
            minZ = std::min(minZ, p.z); maxZ = std::max(maxZ, p.z);
        }
        std::string id = "box" + std::to_string(k);
        viewer.addCube(minX, maxX, minY, maxY, minZ, maxZ, 1.0, 1.0, 1.0, id);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    }
}
