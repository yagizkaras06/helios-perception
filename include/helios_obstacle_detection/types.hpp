// Common point / cloud type aliases shared by all pipeline stages.
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace helios_obstacle_detection
{

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

}  // namespace helios_obstacle_detection
