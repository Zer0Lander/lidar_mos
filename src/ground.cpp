#include "ground.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>

namespace {

// Least-squares plane through the given points, via PCA: the normal is the direction of least spread (smallest-eigenvalue eigenvector of the covariance).
void fitPlane(const CloudI::Ptr& cloud, const std::vector<int>& idxs, Eigen::Vector3f& normal, float& d) {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int i : idxs) {
        centroid += Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
    centroid /= static_cast<float>(idxs.size());

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int i : idxs) {
        Eigen::Vector3f diff(cloud->points[i].x - centroid.x(), cloud->points[i].y - centroid.y(), cloud->points[i].z - centroid.z());
        cov += diff * diff.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    normal = solver.eigenvectors().col(0);
    d = -normal.dot(centroid);
}

// Ground Plane Fitting within one cell: seed from the low points, fit a plane, refine it, and return the cell's ground points. 
// Empty if the cell has no clear near-horizontal ground (e.g. a cell filled by a wall).
std::vector<int> groundInCell(const CloudI::Ptr& cloud, std::vector<int> cell, float distThreshold) {
    const float seedBand = 0.5f;
    const int numIter = 3;
    const size_t minPts = 10;
    const float uprightCos = 0.9f;   // accept the plane only if its normal is within ~25 deg of vertical

    if (cell.size() < minPts) return {};

    std::sort(cell.begin(), cell.end(), [&](int a, int b) { return cloud->points[a].z < cloud->points[b].z; });

    // Robust local ground height: mean z over the cell's 5th-25th percentile band.
    size_t lo = std::max<size_t>(1, cell.size() / 20);
    size_t hi = std::max<size_t>(lo + 1, cell.size() / 4);
    float lpr = 0.0f;
    for (size_t i = lo; i < hi; ++i) lpr += cloud->points[cell[i]].z;
    lpr /= static_cast<float>(hi - lo);

    std::vector<int> seeds;
    for (int idx : cell) {
        float z = cloud->points[idx].z;
        if (z > lpr - seedBand && z < lpr + seedBand) seeds.push_back(idx);
    }
    if (seeds.size() < 3) return {};

    Eigen::Vector3f normal;
    float d;
    for (int iter = 0; iter < numIter; ++iter) {
        fitPlane(cloud, seeds, normal, d);
        std::vector<int> next;
        for (int idx : cell) {
            const auto& p = cloud->points[idx];
            float dist = normal.dot(Eigen::Vector3f(p.x, p.y, p.z)) + d;
            if (std::abs(dist) < distThreshold) next.push_back(idx);
        }
        seeds = std::move(next);
        if (seeds.size() < 3) return {};
    }

    if (std::abs(normal.z()) < uprightCos) return {};
    return seeds;
}

}  // namespace

// Concentric-zone ground segmentation: bin the scan into range rings and angular sectors, and fit the ground locally in each cell. 
// Local fitting handles slope (each cell is nearly flat) and clutter (a cell's own ground dominates it).
pcl::PointIndices::Ptr segmentGround(const CloudI::Ptr& cloud, float distThreshold) {
    const int numSectors = 16;
    const float ringEdges[] = {2.0f, 10.0f, 20.0f, 35.0f, 60.0f};
    const int numRings = sizeof(ringEdges) / sizeof(float) - 1;
    const float twoPi = 2.0f * std::numbers::pi_v<float>;

    std::vector<std::vector<int>> cells(numRings * numSectors);
    for (size_t i = 0; i < cloud->size(); ++i) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float r = std::sqrt(x * x + y * y);

        int ring = -1;
        for (int k = 0; k < numRings; ++k) {
            if (r >= ringEdges[k] && r < ringEdges[k + 1]) { ring = k; break; }
        }
        if (ring < 0) continue;

        float theta = std::atan2(y, x) + std::numbers::pi_v<float>;   // [0, 2pi)
        int sector = std::min(numSectors - 1, static_cast<int>(theta / twoPi * numSectors));
        cells[ring * numSectors + sector].push_back(static_cast<int>(i));
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto& cell : cells) {
        std::vector<int> ground = groundInCell(cloud, std::move(cell), distThreshold);
        inliers->indices.insert(inliers->indices.end(), ground.begin(), ground.end());
    }
    return inliers;
}

CloudI::Ptr removeGround(const CloudI::Ptr& cloud, float distThreshold) {
    pcl::PointIndices::Ptr inliers = segmentGround(cloud, distThreshold);

    CloudI::Ptr out(new CloudI);
    if (inliers->indices.empty()) {
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
