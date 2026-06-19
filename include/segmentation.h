#pragma once
#include "types.h"

namespace Segmentation {

    struct SegmentResult {
        CloudPtr obstacles;
        CloudPtr ground;
    };

    // RANSAC plane fitting → separate ground from obstacles
    SegmentResult removeGround(const CloudPtr& cloud);

} // namespace Segmentation
