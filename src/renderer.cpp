// ─────────────────────────────────────────────
//  src/renderer.cpp
// ─────────────────────────────────────────────

#include "renderer.h"
#include "config.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace fs = std::filesystem;

// ── Color helpers ─────────────────────────────────────────────
static pcl::visualization::PointCloudColorHandlerCustom<PointT>
makeColor(const CloudPtr& cloud, uint8_t r, uint8_t g, uint8_t b) {
    return pcl::visualization::PointCloudColorHandlerCustom<PointT>(
        cloud, r, g, b);
}

// Palette matching clustering.cpp
static const std::vector<std::array<uint8_t,3>> PALETTE = {
    {255, 50,  50 }, {50,  150, 255}, {50,  255, 100},
    {255, 200,  0 }, {200, 50,  255}, {0,   230, 230},
    {255, 128,  0 }, {255, 100, 200}, {150, 255, 50 },
    {50,  100, 255}, {255, 180, 100}, {100, 255, 230},
    {230, 130, 255}, {255, 255, 100}, {100, 200, 100},
    {255, 75,  150}, {150, 100, 255}, {200, 255, 150},
    {100, 150, 200}, {255, 150, 150}
};

namespace Renderer {

// ── Build cube wireframe from BoundingBox ─────────────────────
static void addBoundingBox(
    pcl::visualization::PCLVisualizer& viewer,
    const BoundingBox& box,
    const std::string& id)
{
    // Draw box as a wireframe cube
    viewer.addCube(
        box.center,
        box.orientation,
        box.dimensions.x(),
        box.dimensions.y(),
        box.dimensions.z(),
        id
    );
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        box.color.r / 255.0,
        box.color.g / 255.0,
        box.color.b / 255.0, id);
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, id);
}

// ── Add text label above a bounding box ──────────────────────
static void addLabel(
    pcl::visualization::PCLVisualizer& viewer,
    const BoundingBox& box,
    const std::string& text,
    const std::string& id)
{
    PointT pos;
    pos.x = box.center.x();
    pos.y = box.center.y();
    pos.z = box.center.z() + box.dimensions.z() / 2.0f + 0.3f;

    viewer.addText3D(text, pos, 0.4,
        box.color.r / 255.0,
        box.color.g / 255.0,
        box.color.b / 255.0, id);
}

// ── Build the viewer with all geometries ──────────────────────
static std::shared_ptr<pcl::visualization::PCLVisualizer>
buildViewer(
    const CloudPtr&                 ground,
    const CloudPtr&                 obstacles,
    const std::vector<CloudPtr>&    clusters,
    const std::vector<BoundingBox>& boxes,
    int                             frameIdx)
{
    std::ostringstream title;
    title << Config::WINDOW_NAME << " | Frame " << std::setw(4)
          << std::setfill('0') << frameIdx;

    auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>(title.str());
    viewer->setBackgroundColor(0.05, 0.05, 0.10);
    viewer->setWindowName(title.str());

    // ── Ground (flat green) ────────────────────────────────
    if (!ground->empty()) {
        auto col = makeColor(ground, 50, 150, 50);
        viewer->addPointCloud<PointT>(ground, col, "ground");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "ground");
    }

    // ── Obstacle points not in any cluster (dim white) ─────
    if (!obstacles->empty()) {
        auto col = makeColor(obstacles, 180, 180, 180);
        viewer->addPointCloud<PointT>(obstacles, col, "obstacles");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            Config::POINT_SIZE, "obstacles");
    }

    // ── Colored clusters ───────────────────────────────────
    for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
        auto [r, g, b] = PALETTE[i % PALETTE.size()];
        auto col = makeColor(clusters[i], r, g, b);
        std::string cid = "cluster_" + std::to_string(i);
        viewer->addPointCloud<PointT>(clusters[i], col, cid);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            Config::POINT_SIZE + 1.0f, cid);
    }

    // ── Bounding boxes + labels ────────────────────────────
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        std::string bid   = "box_"   + std::to_string(i);
        std::string lbid  = "label_" + std::to_string(i);

        addBoundingBox(*viewer, boxes[i], bid);

        float dist = boxes[i].center.head<2>().norm();
        std::ostringstream lbl;
        lbl << BBox::classify(boxes[i])
            << " " << std::fixed << std::setprecision(1) << dist << "m";
        addLabel(*viewer, boxes[i], lbl.str(), lbid);
    }

    // ── Coordinate frame ──────────────────────────────────
    viewer->addCoordinateSystem(2.0, "origin");

    // ── Camera position (front elevated view) ─────────────
    viewer->initCameraParameters();
    viewer->setCameraPosition(
         0.0, -30.0, 15.0,   // eye
         0.0,   0.0,  0.0,   // target
         0.0,   0.0,  1.0    // up
    );

    return viewer;
}

void renderFrame(
    const CloudPtr&                 ground,
    const CloudPtr&                 obstacles,
    const std::vector<CloudPtr>&    clusters,
    const std::vector<BoundingBox>& boxes,
    int                             frameIdx,
    bool                            save,
    bool                            block)
{
    auto viewer = buildViewer(ground, obstacles, clusters, boxes, frameIdx);

    if (save) {
        fs::create_directories(Config::OUTPUT_DIR);
        std::ostringstream path;
        path << Config::OUTPUT_DIR << "/frame_"
             << std::setw(4) << std::setfill('0') << frameIdx << ".png";

        // Render one frame and save
        viewer->spinOnce(100);
        viewer->saveScreenshot(path.str());
        std::cout << "  [Render] saved → " << path.str() << "\n";
    }

    if (block) {
        while (!viewer->wasStopped()) {
            viewer->spinOnce(10);
        }
    }
}

} // namespace Renderer
