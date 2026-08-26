#include "io.hpp"

#include <fstream>
#include <iostream>

bool loadScan(const std::string& path, CloudI::Ptr& cloud) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "could not open scan: " << path << "\n";
        return false;
    }
    cloud->reserve(140000);
    // KITTI .bin is raw float32, 4 per point (x, y, z, intensity), no header.
    float buf[4];
    while (file.read(reinterpret_cast<char*>(buf), 4 * sizeof(float))) {
        pcl::PointXYZI p;
        p.x = buf[0];
        p.y = buf[1];
        p.z = buf[2];
        p.intensity = buf[3];
        cloud->push_back(p);
    }
    return true;
}
