#pragma once

#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "types.hpp"

CloudRGB::Ptr colorGround(const CloudI::Ptr& cloud, const pcl::PointIndices& ground);
CloudRGB::Ptr colorByClusters(const CloudI::Ptr& cloud, const std::vector<pcl::PointIndices>& clusters);
void drawBoxes(pcl::visualization::PCLVisualizer& viewer, const CloudI::Ptr& cloud, const std::vector<pcl::PointIndices>& clusters);
