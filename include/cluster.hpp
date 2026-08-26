#pragma once

#include <vector>
#include <pcl/PointIndices.h>
#include "types.hpp"

std::vector<pcl::PointIndices> clusterObjects(const CloudI::Ptr& cloud, float tolerance, int minSize, int maxSize);
