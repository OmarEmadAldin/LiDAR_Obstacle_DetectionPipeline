#pragma once
#include "types.h"
#include <vector>

namespace Clustering {

  
    std::vector<CloudPtr> euclideanClustering(const CloudPtr& obstacles);

} // namespace Clustering
