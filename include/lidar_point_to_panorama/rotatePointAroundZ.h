
#ifndef ROTATEPOINTAROUNDZ_H
#define ROTATEPOINTAROUNDZ_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_point_to_panorama {

/**
 * Rotate a point around the Z axis.
 * @param point The point to rotate.
 * @param angle_degrees The rotation angle in degrees.
 * @return Rotated point.
 */
pcl::PointXYZI rotatePointAroundZ(const pcl::PointXYZI& point, float angle_degrees);

}  // namespace lidar_point_to_panorama

#endif // ROTATEPOINTAROUNDZ_H