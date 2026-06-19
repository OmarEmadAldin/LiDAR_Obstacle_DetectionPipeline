#pragma once
#include "types.h"
#include <vector>

namespace BBox {

    // Fit one OBB per cluster, filter by real-world size
    std::vector<BoundingBox> fitBoxes(const std::vector<CloudPtr>& clusters);

    void printDetections(const std::vector<BoundingBox>& boxes);
    std::string classify(const BoundingBox& box);

} 
