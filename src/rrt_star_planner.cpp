
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

using namespace Eigen;
using namespace std;



class Node
{
public:
    Node(octomap::point3d point):cor(point)
    {
        return;
    }
    Node(){}
    octomap::point3d getcoordinate()
    {
        //ROS_INFO("-------------node.cor: %f  %f  %f",cor.x(),cor.y(),cor.z());
        return cor;
    }
    octomap::point3d setcoordinate(octomap::point3d point)
    {
        cor=point;
    }
    octomap::point3d setparentnode(Node* temp)
    {
        parent_node=temp;
    }
    Node* getparentnode()
    {
        return parent_node;
    }

private:
    octomap::point3d cor;
public:
    Node* parent_node;

};

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;
ros::Time last_wp_time_;

octomap::OcTree* octree_;

std::vector<Node*> node_list;
octomap::point3d currrent_point(1,2,3);
double STEPSIZE=0.5;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
//    distmap.initializeEmpty(1,1,1,true);
    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    currrent_point.x()=current_p(0);
    currrent_point.y()=current_p(1);
    currrent_point.z()=current_p(2);

    //ROS_INFO("X:%f Y:%f  Z:%f",-current_p(0),current_p(1),current_p(2));
	static tf::TransformBroadcaster br;

	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z) );

	geometry_msgs:: Quaternion quat_msg=msg->pose.orientation;
	tf::Quaternion q1(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w) ;

	tf::Quaternion q2(-0.5,0.5,-0.5,0.5);

	tf::Quaternion q=q1*q2;

	transform.setRotation(q);
	// transform.setRotation(q);
	// ROS_INFO("q: %f %f %f %f",q.x(),q.y(),q.z(),q.w());
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_link"));


}

void octo_cb(const octomap_msgs::Octomap& msg)
{
    ros::Time current = ros::Time::now();

    double mapupdate_dt_=0.5;
    if ((current - last_wp_time_).toSec() < mapupdate_dt_) 
    {
        return;
    }
    octomap::AbstractOcTree* abstarcttree = octomap_msgs::msgToMap(msg);

    last_wp_time_ = ros::Time::now();

    if (octree_) 
    {
        delete octree_;
    }
    octree_ = dynamic_cast<octomap::OcTree*>(abstarcttree);
    double octree_resolution_ = octree_->getResolution();

    double x,y,z;
    octree_->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    // std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    octree_->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    // std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

    // ROS_INFO("octree_->getNumLeafNodes(): %ld", octree_->getNumLeafNodes());
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

double findnearestnode_index(octomap::point3d point_rand)
{
    double min_dis=1000;
    double tmp;
    for (int i=0;i<node_list.size();i++)
    {
        if((node_list[i]->getcoordinate()-point_rand).norm()<min_dis)
        {
            tmp=i;
            min_dis=(node_list[i]->getcoordinate()-point_rand).norm();
        }
    }
    return tmp;
}

void rrt_star_gp()
{
    octomap::point3d goal_point (3.0, 0.0, 1.5);

    double x,y,z;
    octree_->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);

    octree_->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);

    octomap::point3d point_rand;
    point_rand.x()=min.x()+rand() / double(RAND_MAX/(max.x() -min.x() ));
    point_rand.y()=min.y()+rand() / double(RAND_MAX/(max.y() -min.y() ));
    point_rand.z()=min.z()+rand() / double(RAND_MAX/(max.z() -min.z() ));

    octomap::OcTreeNode* result = octree_->search (point_rand);
    if(result)
    {
        result->getOccupancy();
        ROS_INFO("x_rand occupancy probility: %f ",result->getOccupancy());
    }

    double x_near_index=findnearestnode_index(point_rand);
    octomap::point3d x_near_cor=node_list[x_near_index]->getcoordinate();
    
    Node* x_new=new Node;
    x_new->setcoordinate((point_rand-x_near_cor)*STEPSIZE+x_near_cor);

    x_new->setparentnode(node_list[0]);
    node_list.push_back(x_new);


    return;

}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "rrt_star_planner");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);
    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::Subscriber octo_sub=nh.subscribe("/octomap_full", 1, octo_cb);

    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);
    double yaw_set = 3.1415926;
    ros::Time last_gp_time_;

    ROS_INFO("pc_tf start!!!!!!!!!");

    while(octree_==nullptr)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("octomap initialize success-----------");

    Node* x_start=new Node;

    x_start->setcoordinate(currrent_point);
    x_start->parent_node=nullptr;
    node_list.push_back(x_start);

    while(ros::ok())
    {
        ros::spinOnce();

        octomap::point3d query (0.10154, 1.002, 1.003);
        
        octomap::OcTreeNode* result = octree_->search (query);
        if(result)
        {
            result->getOccupancy();
            //ROS_INFO("occupancy probility: %f ",result->getOccupancy());
        }

        if((ros::Time::now() - last_gp_time_).toSec()>0.5)
        {
            rrt_star_gp();
            last_gp_time_=ros::Time::now();
        }

    }
	loop_rate.sleep();

    return 0;
}


