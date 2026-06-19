#pragma once
#include "types.h"
#include "bounding_box.h"
#include <vector>
#include <string>

namespace Renderer {

    void renderFrame(
        const CloudPtr&                  ground,
        const CloudPtr&                  obstacles,
        const std::vector<CloudPtr>&     clusters,
        const std::vector<BoundingBox>&  boxes,
        int                              frameIdx,
        bool                             save  = false,
        bool                             block = true
    );

} // namespace Renderer
