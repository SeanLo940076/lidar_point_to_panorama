
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "PointCloudToPanorama.h"
#include "pointfilter.h"
#include "rotatePointAroundZ.h"

ros::Publisher img_pub;

// Parameters for the rotatePointAroundZ function
float rotation_angle;

// Parameters for the pointfilter function
float minDistance, maxDistance, minHeight, maxHeight, minIntensity, maxIntensity;

// Parameters for the convertPointCloudToPanorama function
float h_res, v_res;
std::pair<float, float> h_fov, v_fov;
int point_radius;
float maxDist;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*inputCloud, *cloud);

    // Rotate points using the rotatePointAroundZ function
    for (auto& point : cloud->points)
    {
        point = lidar_point_to_panorama::rotatePointAroundZ(point, rotation_angle);
    }

    // Filter the point cloud using the pointfilter function
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = lidar_point_to_panorama::filterPointCloud(cloud, minDistance, maxDistance, minHeight, maxHeight, minIntensity, maxIntensity);

    // Convert the filtered PointCloud to Panorama
    cv::Mat panorama = lidar_point_to_panorama::convertPointCloudToPanorama(filtered_cloud, h_res, v_res, h_fov, v_fov, point_radius, maxDist);

    // Convert cv::Mat to sensor_msgs/Image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panorama).toImageMsg();

    // Publish the image
    img_pub.publish(msg);
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "panorama_node");
    ros::NodeHandle nh;

    std::string input_topic_name, output_topic_name;
    nh.getParam("panorama_node_input_topic", input_topic_name);
    nh.getParam("panorama_node_output_topic", output_topic_name);

    // Load parameters from the parameter server
    nh.param<float>("rotation_angle", rotation_angle, 0.0);
    nh.param<float>("minDistance", minDistance, 1.0);
    nh.param<float>("maxDistance", maxDistance, 30.0);
    nh.param<float>("minHeight", minHeight, -1.70);
    nh.param<float>("maxHeight", maxHeight, 2.15);
    nh.param<float>("minIntensity", minIntensity, 0.01);
    nh.param<float>("maxIntensity", maxIntensity, 255.0);
    nh.param<float>("h_res", h_res, 0.2);
    nh.param<float>("v_res", v_res, 1.33);
    nh.param<float>("h_fov_min", h_fov.first, -180.0);
    nh.param<float>("h_fov_max", h_fov.second, 180.0);
    nh.param<float>("v_fov_min", v_fov.first, -10.0);
    nh.param<float>("v_fov_max", v_fov.second, 10.0);
    nh.param<int>("point_radius", point_radius, 1);
    nh.param<float>("maxDist", maxDist, 50.0);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 1, cloudCallback); // velodyne_points
    img_pub = nh.advertise<sensor_msgs::Image>(output_topic_name, 1);

    ros::spin();

    return 0;
}
