#pragma once
#include "types.h"
#include <vector>
#include <string>

namespace Loader {

    // Returns sorted list of .bin file paths inside dir
    std::vector<std::string> getBinFiles(const std::string& dir);

    // Load a single KITTI Velodyne .bin file
    // Format: float32[4] per point → x, y, z, intensity
    CloudPtr loadBin(const std::string& filepath);

    // Print basic cloud stats
    void printInfo(const CloudPtr& cloud, const std::string& label = "Cloud");

} // namespace Loader
