#include "pointfilter.h"

namespace lidar_point_to_panorama {
        
pcl::PointCloud<pcl::PointXYZI>::Ptr filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
    float minDistance,
    float maxDistance,
    float minHeight,
    float maxHeight,
    float minIntensity,
    float maxIntensity
) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    for (const auto& point : inputCloud->points) {
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        
        if (distance >= minDistance && distance <= maxDistance && 
            point.z >= minHeight && point.z <= maxHeight &&
            point.intensity >= minIntensity && point.intensity <= maxIntensity) {
            filteredCloud->points.push_back(point);
        }
    }
    
    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;
    filteredCloud->is_dense = true;

    return filteredCloud;
}

}  // namespace lidar_point_to_panorama