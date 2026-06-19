// ─────────────────────────────────────────────
//  src/filter.cpp
// ─────────────────────────────────────────────

#include "filter.h"
#include "config.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

namespace Filter {

CloudPtr voxelDownsample(const CloudPtr& cloud) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(Config::VOXEL_SIZE,
                   Config::VOXEL_SIZE,
                   Config::VOXEL_SIZE);

    CloudPtr filtered(new CloudT);
    vg.filter(*filtered);

    std::cout << "  [Voxel]  " << cloud->size()
              << " → " << filtered->size() << " points\n";
    return filtered;
}

CloudPtr roiCrop(const CloudPtr& cloud) {
    // ── 1. Axis-aligned ROI crop ──────────────────────────
    pcl::CropBox<PointT> roi;
    roi.setMin(Eigen::Vector4f(Config::ROI_MIN_X, Config::ROI_MIN_Y,
                                Config::ROI_MIN_Z, 1.0f));
    roi.setMax(Eigen::Vector4f(Config::ROI_MAX_X, Config::ROI_MAX_Y,
                                Config::ROI_MAX_Z, 1.0f));
    roi.setInputCloud(cloud);

    CloudPtr cropped(new CloudT);
    roi.filter(*cropped);

    // ── 2. Remove ego-vehicle roof points (sphere at origin) ──
    pcl::CropBox<PointT> ego;
    ego.setMin(Eigen::Vector4f(-Config::EGO_RADIUS, -Config::EGO_RADIUS,
                                -Config::EGO_RADIUS, 1.0f));
    ego.setMax(Eigen::Vector4f( Config::EGO_RADIUS,  Config::EGO_RADIUS,
                                 Config::EGO_RADIUS, 1.0f));
    ego.setInputCloud(cropped);
    ego.setNegative(true);   // keep points OUTSIDE the box

    CloudPtr result(new CloudT);
    ego.filter(*result);

    std::cout << "  [ROI]    cropped to " << result->size() << " points\n";
    return result;
}

} // namespace Filter
