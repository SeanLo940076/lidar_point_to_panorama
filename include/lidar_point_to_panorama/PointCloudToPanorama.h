
#ifndef POINTCLOUDTOPANORAMA_H
#define POINTCLOUDTOPANORAMA_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <vector>

namespace lidar_point_to_panorama {

// Convert the 3D point cloud data to a 2D panoramic image
cv::Mat convertPointCloudToPanorama(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud, 
                                    float h_res = 0.2,
                                    float v_res = 1.33,
                                    std::pair<float, float> h_fov = {-180.0, 180.0},
                                    std::pair<float, float> v_fov = {-10.0, 10.0},
                                    int point_radius = 1, float maxDist = 50);

}  // namespace lidar_point_to_panorama

#endif // POINTCLOUDTOPANORAMA_H
