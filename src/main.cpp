// ─────────────────────────────────────────────
//  src/main.cpp  –  LiDAR Obstacle Detection Pipeline (PCL)
//
//  Usage:
//    ./lidar_pipeline                  process frame 0
//    ./lidar_pipeline --frame 5        process frame 5
//    ./lidar_pipeline --all            process all frames sequentially
//    ./lidar_pipeline --all --save     save PNG per frame, no window block
//    ./lidar_pipeline --frame 0 --save save frame 0 PNG + show window
// ─────────────────────────────────────────────

#include "config.h"
#include "loader.h"
#include "filter.h"
#include "segmentation.h"
#include "clustering.h"
#include "bounding_box.h"
#include "renderer.h"

#include <iostream>
#include <string>
#include <chrono>
#include <stdexcept>

// ── Simple argument parser ────────────────────────────────────
struct Args {
    int  frameIdx = 0;
    bool processAll = false;
    bool save = false;
};

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--all")   { a.processAll = true; }
        if (arg == "--save")  { a.save = true; }
        if (arg == "--frame" && i + 1 < argc)
            a.frameIdx = std::stoi(argv[++i]);
    }
    return a;
}

// ── Full pipeline for one frame ───────────────────────────────
void processFrame(
    const std::string& filepath,
    int                frameIdx,
    bool               save,
    bool               block)
{
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  Frame " << frameIdx << "  →  " << filepath << "\n";
    std::cout << std::string(60, '=') << "\n";

    auto t0 = std::chrono::high_resolution_clock::now();

    // ── 1. Load ──────────────────────────────────────────────
    CloudPtr cloud = Loader::loadBin(filepath);
    Loader::printInfo(cloud, "Loaded");

    // ── 2. Voxel Grid Downsampling ───────────────────────────
    cloud = Filter::voxelDownsample(cloud);

    // ── 3. ROI Crop ──────────────────────────────────────────
    cloud = Filter::roiCrop(cloud);

    // ── 4. RANSAC Ground Segmentation ────────────────────────
    auto [obstacles, ground] = Segmentation::removeGround(cloud);

    // ── 5. Euclidean Clustering (KD-Tree) ────────────────────
    auto clusters = Clustering::euclideanClustering(obstacles);

    // ── 6. Bounding Boxes + Classification ───────────────────
    auto boxes = BBox::fitBoxes(clusters);
    BBox::printDetections(boxes);

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "  ⏱  Pipeline time: " << ms << " ms\n";

    // ── 7. Render ────────────────────────────────────────────
    Renderer::renderFrame(ground, obstacles, clusters, boxes,
                          frameIdx, save, block);
}

// ── Entry point ───────────────────────────────────────────────
int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    try {
        auto files = Loader::getBinFiles(Config::DATA_DIR);
        std::cout << "Found " << files.size()
                  << " .bin files in '" << Config::DATA_DIR << "'\n";

        if (args.processAll) {
            // block=false so frames advance automatically
            // save=true  → writes a PNG per frame
            for (int i = 0; i < static_cast<int>(files.size()); ++i)
                processFrame(files[i], i, args.save, /*block=*/false);
        } else {
            if (args.frameIdx >= static_cast<int>(files.size())) {
                std::cerr << "Error: frame " << args.frameIdx
                          << " not found (total=" << files.size() << ")\n";
                return 1;
            }
            processFrame(files[args.frameIdx], args.frameIdx,
                         args.save, /*block=*/true);
        }

    } catch (const std::exception& e) {
        std::cerr << "\nFATAL: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
