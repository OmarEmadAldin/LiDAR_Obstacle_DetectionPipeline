#include "bounding_box.h"
#include "config.h"
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <iomanip>
#include <algorithm>

// Palette (must match clustering.cpp ordering)
static const std::vector<std::array<uint8_t,3>> PALETTE = {
    {255, 50,  50 }, {50,  150, 255}, {50,  255, 100},
    {255, 200,  0 }, {200, 50,  255}, {0,   230, 230},
    {255, 128,  0 }, {255, 100, 200}, {150, 255, 50 },
    {50,  100, 255}, {255, 180, 100}, {100, 255, 230},
    {230, 130, 255}, {255, 255, 100}, {100, 200, 100},
    {255, 75,  150}, {150, 100, 255}, {200, 255, 150},
    {100, 150, 200}, {255, 150, 150}
};

namespace BBox {

static bool isValidBox(const BoundingBox& box) {
    auto dims = box.dimensions;
    auto sorted = std::array<float,3>{dims.x(), dims.y(), dims.z()};
    std::sort(sorted.begin(), sorted.end()); // [h, w, l]

    return (sorted[0] >= Config::BOX_MIN_H && sorted[0] <= Config::BOX_MAX_H &&
            sorted[1] >= Config::BOX_MIN_W && sorted[1] <= Config::BOX_MAX_W &&
            sorted[2] >= Config::BOX_MIN_D && sorted[2] <= Config::BOX_MAX_D);
}

std::vector<BoundingBox> fitBoxes(const std::vector<CloudPtr>& clusters) {
    std::vector<BoundingBox> boxes;
    boxes.reserve(clusters.size());

    for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
        const auto& cluster = clusters[i];
        if (cluster->size() < 4) continue;

        // ── PCA to find principal axes (Oriented Bounding Box) ──
        pcl::PCA<PointT> pca;
        pca.setInputCloud(cluster);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
        Eigen::Vector3f eigenValues  = pca.getEigenValues();

        // Project cluster onto PCA axes
        CloudPtr projected(new CloudT);
        pca.project(*cluster, *projected);

        PointT minPt, maxPt;
        pcl::getMinMax3D(*projected, minPt, maxPt);

        // Dimensions in PCA space
        Eigen::Vector3f dims(
            maxPt.x - minPt.x,
            maxPt.y - minPt.y,
            maxPt.z - minPt.z
        );

        // Center in PCA space → back to world space
        Eigen::Vector3f centerPCA(
            (maxPt.x + minPt.x) / 2.0f,
            (maxPt.y + minPt.y) / 2.0f,
            (maxPt.z + minPt.z) / 2.0f
        );
        Eigen::Vector3f centerWorld =
            eigenVectors * centerPCA +
            Eigen::Vector3f(centroid.x(), centroid.y(), centroid.z());

        // Rotation from PCA axes
        Eigen::Quaternionf q(eigenVectors);
        q.normalize();

        // Assign palette color
        auto [r, g, b] = PALETTE[i % PALETTE.size()];
        pcl::RGB color;
        color.r = r; color.g = g; color.b = b;

        BoundingBox box;
        box.center      = centerWorld;
        box.dimensions  = dims;
        box.orientation = q;
        box.color       = color;

        if (isValidBox(box))
            boxes.push_back(box);
    }

    std::cout << "  [BBox]   " << boxes.size()
              << " bounding boxes after size filtering\n";
    return boxes;
}

void printDetections(const std::vector<BoundingBox>& boxes) {
    if (boxes.empty()) {
        std::cout << "  No obstacles detected.\n\n";
        return;
    }
    std::cout << "\n  " << std::left
              << std::setw(4)  << "#"
              << std::setw(32) << "Center (x, y, z)"
              << std::setw(8)  << "W"
              << std::setw(8)  << "D"
              << std::setw(8)  << "H"
              << std::setw(12) << "Class"
              << std::setw(8)  << "Dist(m)"
              << "\n  " << std::string(72, '-') << "\n";

    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        const auto& b = boxes[i];
        float dist = b.center.head<2>().norm();  // XY distance from sensor

        std::ostringstream centerStr;
        centerStr << std::fixed << std::setprecision(1)
                  << "(" << b.center.x() << ", "
                  << b.center.y() << ", "
                  << b.center.z() << ")";

        std::cout << "  " << std::left
                  << std::setw(4)  << i
                  << std::setw(32) << centerStr.str()
                  << std::setw(8)  << std::fixed << std::setprecision(2) << b.dimensions.x()
                  << std::setw(8)  << b.dimensions.y()
                  << std::setw(8)  << b.dimensions.z()
                  << std::setw(12) << classify(b)
                  << std::setw(8)  << dist
                  << "\n";
    }
    std::cout << "\n";
}

std::string classify(const BoundingBox& box) {
    auto d = box.dimensions;
    auto sorted = std::array<float,3>{d.x(), d.y(), d.z()};
    std::sort(sorted.begin(), sorted.end()); // [h, w, l]
    float h = sorted[0], w = sorted[1], l = sorted[2];

    if (l > 3.5f && w > 1.5f)          return "Car";
    if (l > 7.0f && w > 2.0f)          return "Truck";
    if (h > 1.5f && w < 1.0f)          return "Pedestrian";
    if (h > 1.0f && l < 2.5f)          return "Cyclist";
    return "Unknown";
}

} 
