
#ifndef POINTFILTER_H
#define POINTFILTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_point_to_panorama {

// Filter the point cloud based on distance, height, and intensity
pcl::PointCloud<pcl::PointXYZI>::Ptr filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
    float minDistance = 1.0,
    float maxDistance = 30.0,
    float minHeight = -1.70,
    float maxHeight = 2.15,
    float minIntensity = 0.01,
    float maxIntensity = 255.0
);

}  // namespace lidar_point_to_panorama

#endif // POINTFILTER_H
