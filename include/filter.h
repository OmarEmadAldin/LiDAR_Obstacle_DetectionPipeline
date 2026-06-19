#pragma once
#include "types.h"

namespace Filter {

    CloudPtr voxelDownsample(const CloudPtr& cloud);
    CloudPtr roiCrop(const CloudPtr& cloud);

} // namespace Filter
