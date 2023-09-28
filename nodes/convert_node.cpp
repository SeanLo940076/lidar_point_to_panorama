#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "PointCloudToPanorama.h"

image_transport::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Retrieve parameters for conversion to panorama
    float h_res, v_res, point_radius, maxDist;
    std::pair<float, float> h_fov, v_fov;
    ros::param::get("h_res", h_res);
    ros::param::get("v_res", v_res);
    ros::param::get("h_fov_min", h_fov.first);
    ros::param::get("h_fov_max", h_fov.second);
    ros::param::get("v_fov_min", v_fov.first);
    ros::param::get("v_fov_max", v_fov.second);
    ros::param::get("point_radius", point_radius);
    ros::param::get("maxDist", maxDist);

    cv::Mat panorama = lidar_point_to_panorama::convertPointCloudToPanorama(cloud, h_res, v_res, h_fov, v_fov, point_radius, maxDist);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panorama).toImageMsg();
    pub.publish(img_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_node");
    ros::NodeHandle nh;

    // Retrieve topic names from the parameter server
    std::string input_topic_name, output_topic_name;
    nh.getParam("convert_node_input_topic", input_topic_name);
    nh.getParam("convert_node_output_topic", output_topic_name);

    image_transport::ImageTransport it(nh);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 1, cloudCallback);
    pub = it.advertise(output_topic_name, 1);

    ros::spin();

    return 0;
}