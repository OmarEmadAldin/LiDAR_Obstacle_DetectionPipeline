#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Point type — XYZI keeps intensity from Velodyne
using PointT  = pcl::PointXYZI;
using CloudT  = pcl::PointCloud<PointT>;
using CloudPtr  = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

// Bounding box struct
struct BoundingBox {
    Eigen::Vector3f center;
    Eigen::Vector3f dimensions;   // (width, depth, height)
    Eigen::Quaternionf orientation;
    pcl::RGB color;
};
