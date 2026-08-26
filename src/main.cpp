#include <iostream>
#include <string>

#include "io.hpp"
#include "preprocess.hpp"
#include "ground.hpp"
#include "cluster.hpp"
#include "viz.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <scan.bin> [ground|clusters|boxes]\n";
        return 1;
    }
    const std::string scanPath = argv[1];
    const std::string mode = (argc > 2) ? argv[2] : "boxes";
    if (mode != "ground" && mode != "clusters" && mode != "boxes") {
        std::cerr << "unknown mode: " << mode << " (use ground, clusters, or boxes)\n";
        return 1;
    }

    CloudI::Ptr cloud(new CloudI);
    if (!loadScan(scanPath, cloud)) {
        return 1;
    }
    CloudI::Ptr down = downsample(cropByRange(cloud, 2.0f, 50.0f), 0.2f);

    pcl::visualization::PCLVisualizer viewer("lidar_mos");
    viewer.setBackgroundColor(0.05, 0.05, 0.05);

    if (mode == "ground") {
        pcl::PointIndices::Ptr ground = segmentGround(down, 0.3f);
        viewer.addPointCloud<pcl::PointXYZRGB>(colorGround(down, *ground), "scene");
    } else {
        CloudI::Ptr objects = removeGround(down, 0.3f);
        auto clusters = clusterObjects(objects, 0.7f, 10, 25000);
        std::cout << "clusters: " << clusters.size() << "\n";
        viewer.addPointCloud<pcl::PointXYZRGB>(colorByClusters(objects, clusters), "scene");
        if (mode == "boxes") {
            drawBoxes(viewer, objects, clusters);
        }
    }

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene");
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
