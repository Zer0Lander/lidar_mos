#include <cstdint>
#include <format>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/search/kdtree.h>

#include "io.hpp"
#include "preprocess.hpp"
#include "ground.hpp"

// SemanticKITTI ground classes: road, parking, sidewalk, other-ground, lane-marking, terrain.
static bool isGround(uint32_t cls) {
    return cls == 40 || cls == 44 || cls == 48 || cls == 49 || cls == 60 || cls == 72;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << std::format("usage: {} <sequence_dir> [start] [end] [step]\n", argv[0]);
        return 1;
    }
    const std::string seq = argv[1];
    const int start = (argc > 2) ? std::stoi(argv[2]) : 0;
    int end = (argc > 3) ? std::stoi(argv[3]) : start;
    const int step = (argc > 4) ? std::stoi(argv[4]) : 1;
    if (end < start) end = start;

    double sumRecall = 0, sumResidual = 0;
    int n = 0;
    double worstResidual = 0;
    int worstFrame = start;

    for (int f = start; f <= end; f += step) {
        const std::string scanPath = std::format("{}/velodyne/{:06d}.bin", seq, f);
        const std::string labelPath = std::format("{}/labels/{:06d}.label", seq, f);

        CloudI::Ptr raw(new CloudI);
        if (!loadScan(scanPath, raw)) continue;
        const std::vector<uint32_t> labels = loadLabels(labelPath);
        if (labels.size() != raw->size()) {
            std::cerr << std::format("skip frame {}: label/scan size mismatch\n", f);
            continue;
        }

        // Ground truth per downsampled point comes from the nearest raw point, since voxel downsampling does not preserve the original indices.
        pcl::search::KdTree<pcl::PointXYZI> tree;
        tree.setInputCloud(raw);

        CloudI::Ptr down = downsample(cropByRange(raw, 2.0f, 50.0f), 0.2f);
        pcl::PointIndices::Ptr inliers = segmentGround(down, 0.3f);

        std::vector<char> predictedGround(down->size(), 0);
        for (int i : inliers->indices) predictedGround[i] = 1;

        long tp = 0, fp = 0, fn = 0, tn = 0;
        std::vector<int> idx(1);
        std::vector<float> sqDist(1);
        for (size_t i = 0; i < down->size(); ++i) {
            tree.nearestKSearch(down->points[i], 1, idx, sqDist);
            const bool truth = idx[0] < static_cast<int>(labels.size()) && isGround(labels[idx[0]]);
            const bool pred = predictedGround[i];
            if (pred && truth) ++tp;
            else if (pred && !truth) ++fp;
            else if (!pred && truth) ++fn;
            else ++tn;
        }

        const double recall = (tp + fn) ? 100.0 * tp / (tp + fn) : 0.0;      // ground removed
        const double precision = (tp + fp) ? 100.0 * tp / (tp + fp) : 0.0;   // of removed, actually ground
        const double residual = (fn + tn) ? 100.0 * fn / (fn + tn) : 0.0;    // ground left among survivors

        sumRecall += recall;
        sumResidual += residual;
        ++n;
        if (residual > worstResidual) { worstResidual = residual; worstFrame = f; }

        std::cout << std::format("frame {:5} | recall {:5.1f}% | precision {:5.1f}% | residual {:5.1f}%\n", f, recall, precision, residual);
    }

    if (n > 0) {
        std::cout << std::format("\n{} frames | mean recall {:.1f}% | mean residual {:.1f}% | worst residual {:.1f}% @ frame {}\n", n, sumRecall / n, sumResidual / n, worstResidual, worstFrame);
    }
    return 0;
}
