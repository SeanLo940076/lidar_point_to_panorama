#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pointfilter.h"

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Retrieve parameters for filtering
    float minDistance, maxDistance, minHeight, maxHeight, minIntensity, maxIntensity;
    ros::param::get("minDistance", minDistance);
    ros::param::get("maxDistance", maxDistance);
    ros::param::get("minHeight", minHeight);
    ros::param::get("maxHeight", maxHeight);
    ros::param::get("minIntensity", minIntensity);
    ros::param::get("maxIntensity", maxIntensity);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = lidar_point_to_panorama::filterPointCloud(cloud, minDistance, maxDistance, minHeight, maxHeight, minIntensity, maxIntensity);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);
    output.header.frame_id = "velodyne";
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh;

    // Retrieve topic names from the parameter server
    std::string input_topic_name, output_topic_name;
    nh.getParam("filter_node_input_topic", input_topic_name);
    nh.getParam("filter_node_output_topic", output_topic_name);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 1);

    ros::spin();

    return 0;
}