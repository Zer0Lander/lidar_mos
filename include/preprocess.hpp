#pragma once

#include "types.hpp"

CloudI::Ptr cropByRange(const CloudI::Ptr& cloud, float minRange, float maxRange);
CloudI::Ptr downsample(const CloudI::Ptr& cloud, float leaf);
