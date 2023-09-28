#include "PointCloudToPanorama.h"

namespace lidar_point_to_panorama {

cv::Mat convertPointCloudToPanorama(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
                                    float h_res, float v_res,
                                    std::pair<float, float> h_fov,
                                    std::pair<float, float> v_fov,
                                    int point_radius, float maxDist)
{

    int x_size = static_cast<int>(std::ceil((h_fov.second - h_fov.first) / h_res));
    int y_size = static_cast<int>(std::ceil((v_fov.second - v_fov.first) / v_res));

    cv::Mat panorama = cv::Mat::zeros(y_size + 1, x_size + 1, CV_8UC3);

    for (const auto &point : pointCloud->points)
    {
        float d = std::sqrt(point.x * point.x + point.y * point.y); // 2D distance using only x and y
        float x_img = std::atan2(-point.y, point.x) / (h_res * (M_PI / 180.0));
        float y_img = -(std::atan2(point.z, d) / (v_res * (M_PI / 180.0)));

        int x_offset = static_cast<int>(h_fov.first / h_res);
        x_img = std::trunc(x_img - x_offset);

        int y_offset = static_cast<int>(v_fov.second / v_res);
        y_img = std::trunc(y_img + y_offset);

        int red, green;
        if (d > maxDist)
        {
            red = 0;
            green = 255;
        }
        else
        {
            double factor = 1.0;  // Or any other value greater than 1 that you prefer
            red = std::min(255, static_cast<int>(255 * pow((1 - d / maxDist), factor)));
            green = std::min(255, static_cast<int>(255 * pow(d / maxDist, factor)));

        }

        for (int dx = -point_radius; dx <= point_radius; dx++)
        {
            for (int dy = -point_radius; dy <= point_radius; dy++)
            {
                int x_pos = static_cast<int>(x_img) + dx;
                int y_pos = static_cast<int>(y_img) + dy;

                // Ensure coordinates are within the image bounds
                if (x_pos >= 0 && x_pos < panorama.cols && y_pos >= 0 && y_pos < panorama.rows)
                {
                    panorama.at<cv::Vec3b>(y_pos, x_pos) = cv::Vec3b(0, green, red);
                }
            }
        }
    }
    return panorama;
}

}
