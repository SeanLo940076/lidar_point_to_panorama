
#include "rotatePointAroundZ.h"
#include <cmath>

namespace lidar_point_to_panorama {

pcl::PointXYZI rotatePointAroundZ(const pcl::PointXYZI& point, float angle_degrees) {
    float theta = angle_degrees * M_PI / 180.0;
    pcl::PointXYZI rotated_point;
    rotated_point.x = point.x * cos(theta) - point.y * sin(theta);
    rotated_point.y = point.x * sin(theta) + point.y * cos(theta);
    rotated_point.z = point.z;
    rotated_point.intensity = point.intensity;
    return rotated_point;
}

}  // namespace lidar_point_to_panorama
