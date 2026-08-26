#pragma once

#include <pcl/PointIndices.h>
#include "types.hpp"

pcl::PointIndices::Ptr segmentGround(const CloudI::Ptr& cloud, float distThreshold);
CloudI::Ptr removeGround(const CloudI::Ptr& cloud, float distThreshold);
