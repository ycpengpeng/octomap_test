

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <time.h> 
#include <stdlib.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace Eigen;
using namespace std;




int main(int argc, char** argv) {
    ros::init(argc, argv, "map_test");
    ros::NodeHandle nh;

    ros::Publisher map_pub = nh.advertise<octomap_msgs::Octomap>("/octomap_full", 1);


    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);

    octomap::AbstractOcTree* tree=octomap::AbstractOcTree::read("/home/pp/PX4-Autopilot/mapfile2.ot");
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);   

    cout<<octree->getResolution()<<endl;


    octomap_msgs::Octomap octomap ;
    octomap.binary = 0 ;
    octomap.id = 1 ;
    octomap.resolution =0.05 ;
    octomap.header.frame_id = "/map";
    octomap.header.stamp = ros::Time::now();
    bool res = octomap_msgs::fullMapToMsg(*octree, octomap);
    //octomap.data = td_vector_to_py_list(octomap.data)  ;

    double x,y,z;
    octree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;

    octomap::point3d point_rand(1.29,-1.046,1.5);
    octomap::OcTreeNode* result = octree->search (point_rand);

    cout<<"occupancy: "<<result->getOccupancy()<<endl;
    ROS_INFO("map_test start!!!!!!!!!");
    while(ros::ok())
    {
        map_pub.publish(octomap) ;
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
