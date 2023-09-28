#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "rotatePointAroundZ.h"

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Retrieve rotation angle from the parameter server
    double rotation_angle;
    ros::param::get("rotation_angle", rotation_angle);

    pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& point : cloud->points) {
        rotated_cloud->points.push_back(lidar_point_to_panorama::rotatePointAroundZ(point, rotation_angle));
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*rotated_cloud, output);
    output.header.frame_id = "velodyne";
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_node");
    ros::NodeHandle nh;

    // Retrieve topic names from the parameter server
    std::string input_topic_name, output_topic_name;
    nh.getParam("rotate_node_input_topic", input_topic_name);
    nh.getParam("rotate_node_output_topic", output_topic_name);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 1);

    ros::spin();

    return 0;
}