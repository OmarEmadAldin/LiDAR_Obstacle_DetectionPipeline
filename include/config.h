#pragma once
// ─────────────────────────────────────────────
//  include/config.h  –  All tunable parameters
// ─────────────────────────────────────────────

#include <string>
#include <array>

namespace Config {

    // ── Paths ─────────────────────────────────────────────
    inline const std::string DATA_DIR   = "/home/omar_ben_emad/PCL/lidar_pcl/data";           // folder with .bin files
    inline const std::string OUTPUT_DIR = "output/frames";  // saved PNG frames

    // ── Voxel Grid ────────────────────────────────────────
    constexpr float VOXEL_SIZE = 0.2f;   // meters

    // ── ROI Crop ──────────────────────────────────────────
    constexpr float ROI_MIN_X = -40.0f,  ROI_MAX_X = 40.0f;
    constexpr float ROI_MIN_Y = -20.0f,  ROI_MAX_Y = 20.0f;
    constexpr float ROI_MIN_Z =  -3.0f,  ROI_MAX_Z =  1.5f;
    constexpr float EGO_RADIUS = 1.5f;   // remove points this close to sensor

    // ── RANSAC Ground Segmentation ────────────────────────
    constexpr double RANSAC_DISTANCE_THRESH = 0.25;
    constexpr int    RANSAC_MAX_ITERATIONS  = 150;

    // ── Euclidean Clustering ──────────────────────────────
    constexpr double CLUSTER_TOLERANCE  = 0.5;    // meters
    constexpr int    CLUSTER_MIN_POINTS = 10;
    constexpr int    CLUSTER_MAX_POINTS = 5000;

    // ── Bounding Box Size Filters (meters) ───────────────
    constexpr float BOX_MIN_W = 0.5f,  BOX_MAX_W = 6.0f;
    constexpr float BOX_MIN_D = 0.5f,  BOX_MAX_D = 6.0f;
    constexpr float BOX_MIN_H = 0.3f,  BOX_MAX_H = 4.0f;

    // ── Renderer ──────────────────────────────────────────
    inline const std::string WINDOW_NAME = "LiDAR Detection Pipeline";
    constexpr int   WIN_WIDTH  = 1280;
    constexpr int   WIN_HEIGHT = 720;
    constexpr float POINT_SIZE = 2.0f;

} // namespace Config
