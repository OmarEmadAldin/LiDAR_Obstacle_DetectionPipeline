// ─────────────────────────────────────────────
//  src/segmentation.cpp
// ─────────────────────────────────────────────

#include "segmentation.h"
#include "config.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

namespace Segmentation {

SegmentResult removeGround(const CloudPtr& cloud) {
    // ── RANSAC plane model fitting ────────────────────────
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(Config::RANSAC_DISTANCE_THRESH);
    seg.setMaxIterations(Config::RANSAC_MAX_ITERATIONS);
    seg.setInputCloud(cloud);

    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coeffs);

    if (inliers->indices.empty()) {
        std::cerr << "  [RANSAC] WARNING: Could not find a plane.\n";
        SegmentResult res;
        res.obstacles = cloud;
        res.ground    = CloudPtr(new CloudT);
        return res;
    }

    std::cout << "  [RANSAC] plane: "
              << coeffs->values[0] << "x + "
              << coeffs->values[1] << "y + "
              << coeffs->values[2] << "z + "
              << coeffs->values[3] << " = 0"
              << "  |  inliers=" << inliers->indices.size() << "\n";

    // ── Extract ground and obstacle clouds ────────────────
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);

    CloudPtr ground(new CloudT);
    extractor.setNegative(false);
    extractor.filter(*ground);

    CloudPtr obstacles(new CloudT);
    extractor.setNegative(true);
    extractor.filter(*obstacles);

    std::cout << "  [RANSAC] obstacles=" << obstacles->size()
              << "  ground=" << ground->size() << "\n";

    return { obstacles, ground };
}

} // namespace Segmentation
