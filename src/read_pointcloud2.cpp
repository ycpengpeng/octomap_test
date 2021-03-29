

#include <ros/ros.h>
#include<pcl/point_cloud.h> 
#include<pcl_conversions/pcl_conversions.h>  
#include<pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;
 
void pointcloud_callback(const sensor_msgs::PointCloud2 &msg)
{
	sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
	for (int i=0; i<out_pointcloud.points.size(); i++) 
    {
        // ROS_INFO("out_pointcloud.points: %f %f %f",out_pointcloud.points[i].x,out_pointcloud.points[i].y,out_pointcloud.points[i].z);
	}
    ROS_INFO("out_pointcloud.points.size(): %ld", out_pointcloud.points.size());

	// ROS_INFO("------------------");  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_pointcloud2");

    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    ros::Subscriber pc_sub = n.subscribe("/camera/depth/points", 1, pointcloud_callback);

    ROS_INFO("-----------Read point cloud-----------");

    
    while(ros::ok())
    {
        ros::spinOnce();

    }
    loop_rate.sleep();

    return 0;
}