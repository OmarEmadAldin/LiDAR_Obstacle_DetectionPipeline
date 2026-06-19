#include "clustering.h"
#include "config.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

// 20 distinct RGB colors for clusters
static const std::vector<std::array<uint8_t,3>> PALETTE = {
    {255, 50,  50 }, {50,  150, 255}, {50,  255, 100},
    {255, 200,  0 }, {200, 50,  255}, {0,   230, 230},
    {255, 128,  0 }, {255, 100, 200}, {150, 255, 50 },
    {50,  100, 255}, {255, 180, 100}, {100, 255, 230},
    {230, 130, 255}, {255, 255, 100}, {100, 200, 100},
    {255, 75,  150}, {150, 100, 255}, {200, 255, 150},
    {100, 150, 200}, {255, 150, 150}
};

namespace Clustering {

std::vector<CloudPtr> euclideanClustering(const CloudPtr& obstacles) {
    if (obstacles->empty()) {
        std::cout << "  [Cluster] No obstacle points.\n";
        return {};
    }

    // ── Build KD-Tree ─────────────────────────────────────
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(obstacles);

    // ── Euclidean Cluster Extraction ──────────────────────
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(Config::CLUSTER_TOLERANCE);
    ec.setMinClusterSize(Config::CLUSTER_MIN_POINTS);
    ec.setMaxClusterSize(Config::CLUSTER_MAX_POINTS);
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacles);
    ec.extract(clusterIndices);

    std::cout << "  [Cluster] found " << clusterIndices.size()
              << " clusters\n";

    // ── Build one colored cloud per cluster ───────────────
    std::vector<CloudPtr> clusters;
    clusters.reserve(clusterIndices.size());

    int idx = 0;
    for (const auto& indices : clusterIndices) {
        CloudPtr cluster(new CloudT);

        auto [r, g, b] = PALETTE[idx % PALETTE.size()];

        for (int i : indices.indices) {
            PointT pt = (*obstacles)[i];
            // Pack color into intensity field for XYZ+I clouds
            // (for visualization we'll handle color in renderer)
            pt.intensity = static_cast<float>(idx);
            cluster->push_back(pt);
        }

        cluster->width    = static_cast<uint32_t>(cluster->size());
        cluster->height   = 1;
        cluster->is_dense = false;
        clusters.push_back(cluster);
        ++idx;
    }

    return clusters;
}

} // namespace Clustering
