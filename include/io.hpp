#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include "types.hpp"

bool loadScan(const std::string& path, CloudI::Ptr& cloud);

// Load a SemanticKITTI .label file as one semantic class id per point (low 16 bits; the instance id in the high 16 bits is dropped).
std::vector<uint32_t> loadLabels(const std::string& path);
