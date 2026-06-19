cmake_minimum_required(VERSION 3.10)
project(lidar_pcl VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# ── Find PCL ─────────────────────────────────────────────────
find_package(PCL 1.10 REQUIRED COMPONENTS
    common
    io
    filters
    segmentation
    kdtree
    features
    visualization
)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(SOURCES
    src/main.cpp
    src/loader.cpp
    src/filter.cpp
    src/segmentation.cpp
    src/clustering.cpp
    src/bounding_box.cpp
    src/renderer.cpp
)

add_executable(lidar_pipeline ${SOURCES})
target_link_libraries(lidar_pipeline ${PCL_LIBRARIES})
install(TARGETS lidar_pipeline DESTINATION bin)
