#include <cfloat>
#include "loader.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace fs = std::filesystem;

namespace Loader {

std::vector<std::string> getBinFiles(const std::string& dir) {
    std::vector<std::string> files;

    if (!fs::exists(dir))
        throw std::runtime_error("Data directory not found: " + dir);

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".bin")
            files.push_back(entry.path().string());
    }

    if (files.empty())
        throw std::runtime_error(
            "No .bin files found in: " + dir +
            "\nDownload a KITTI sequence and place .bin files there.");

    std::sort(files.begin(), files.end());
    return files;
}

CloudPtr loadBin(const std::string& filepath) {
    // Each point = 4 floats: x, y, z, intensity
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("Cannot open: " + filepath);

    // Get file size → number of points
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t numPoints = fileSize / (4 * sizeof(float));

    CloudPtr cloud(new CloudT);
    cloud->reserve(numPoints);

    float buf[4];
    while (file.read(reinterpret_cast<char*>(buf), sizeof(buf))) {
        PointT pt;
        pt.x         = buf[0];
        pt.y         = buf[1];
        pt.z         = buf[2];
        pt.intensity = buf[3];
        cloud->push_back(pt);
    }

    cloud->width    = static_cast<uint32_t>(cloud->size());
    cloud->height   = 1;
    cloud->is_dense = false;

    return cloud;
}

void printInfo(const CloudPtr& cloud, const std::string& label) {
    float xMin = FLT_MAX, xMax = -FLT_MAX;
    float yMin = FLT_MAX, yMax = -FLT_MAX;
    float zMin = FLT_MAX, zMax = -FLT_MAX;

    for (const auto& pt : *cloud) {
        xMin = std::min(xMin, pt.x);  xMax = std::max(xMax, pt.x);
        yMin = std::min(yMin, pt.y);  yMax = std::max(yMax, pt.y);
        zMin = std::min(zMin, pt.z);  zMax = std::max(zMax, pt.z);
    }

    std::cout << "  [" << label << "]"
              << "  points=" << cloud->size()
              << "  x=[" << xMin << ", " << xMax << "]"
              << "  y=[" << yMin << ", " << yMax << "]"
              << "  z=[" << zMin << ", " << zMax << "]\n";
}

} // namespace Loader
