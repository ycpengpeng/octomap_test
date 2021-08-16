
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
#include <minimum_snap_path_plan.h>


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

    void printcoordinate()
    {
        ROS_INFO("node.cor------------: %f  %f  %f",cor.x(),cor.y(),cor.z());
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
    float node_cost;

};

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;
ros::Time last_wp_time_;

octomap::OcTree* octree_;

std::vector<Node*> node_list;
octomap::point3d currrent_point(1,2,3);
double STEPSIZE=0.5; //////////////////////////////////////////////////////////////////////////
visualization_msgs::Marker marker;
visualization_msgs::Marker marker1;

ros::Publisher marker_pub;

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

bool check_collision_free(octomap::point3d x_near_cor,octomap::point3d x_new_cor)
{
    bool free=true;
    ros::Time start;
    start=ros::Time::now();
    float square=0.8;

    std::vector<octomap::point3d> offset;
    octomap::point3d offset1(-square/2,square/2,0);
    offset.push_back(offset1);
    octomap::point3d offset2(square/2,square/2,0);
    offset.push_back(offset2);
    octomap::point3d offset3(square/2,-square/2,0);
    offset.push_back(offset3);
    octomap::point3d offset4(-square/2,-square/2,0);
    offset.push_back(offset4);


    int count=0;
    while(count<=3)
    {
        octomap::point3d x_near_offset=x_near_cor+offset[count];
        octomap::point3d x_new_offset=x_new_cor+offset[count];
        octomap::point3d x_check=x_near_offset;
        int i=0;

        octomap::point3d x_direction=(x_new_offset-x_near_offset).normalized();
        ROS_INFO("-----------------------");
        ROS_INFO("x_near_offset: %f %f %f x_new_offset: %f %f %f",x_near_offset.x(),x_near_offset.y(),x_near_offset.z(),
                x_new_offset.x(),x_new_offset.y(),x_new_offset.z() );
        while(x_near_offset.distance(x_check)<x_near_offset.distance(x_new_offset))
        {
            x_check=x_near_offset+x_direction*0.05*i;
            octomap::OcTreeNode* result = octree_->search (x_check);
            ROS_INFO("x_check: %f %f %f",x_check.x(),x_check.y(),x_check.z());
            if (result)
            {
                // ROS_INFO("occupancy probability: %f",result->getOccupancy());
                
                if(result->getOccupancy()>0.5)
                {
                    ROS_WARN("there is collision-------");
                    ROS_INFO("OCCUPANCY: %f",result->getOccupancy());
                    return false;
                }
            }
            i=i+1;
        }
        count++;
    }

    ROS_WARN("check collision free-------------  ");
    return free;
}


bool rrt_star_gp()
{
    octomap::point3d goal_point (7.0, 0.0, 1.5);  ///////////////////////////////////////////
    
    // ROS_INFO("goal_point: %f %f %f",goal_point.x(),goal_point.y(),goal_point.z());

    double x,y,z;
    octree_->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);

    octree_->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);

    octomap::point3d point_rand;

    double rand_tmp=rand()%100;
    // ROS_INFO("goal_point: %f %f %f",goal_point.x(),goal_point.y(),goal_point.z());

    if(rand_tmp<90)
    {
        min.x()=0;
        // min.y()=0;
        min.z()=1.5;
        max.z()=1.5;
        point_rand.x()=min.x()+rand() / double(RAND_MAX/(max.x() -min.x() ));
        point_rand.y()=min.y()+rand() / double(RAND_MAX/(max.y() -min.y() ));
        point_rand.z()=min.z()+rand() / double(RAND_MAX/(max.z() -min.z() ));
    }
    else{
        point_rand=goal_point;
    }
   // ROS_INFO("point_rand: %f %f %f",point_rand.x(),point_rand.y(),point_rand.z());

    octomap::OcTreeNode* result = octree_->search (point_rand);
    if(result)
    {

        //ROS_INFO("point_rand: %f %f %f",point_rand.x(),point_rand.y(),point_rand.z());
        //ROS_INFO("x_rand occupancy probility: %f ",result->getOccupancy());
        //ROS_INFO("///////////////////////");
        double x_near_index=findnearestnode_index(point_rand);
        octomap::point3d x_near_cor=node_list[x_near_index]->getcoordinate();

        octomap::point3d x_direction=(point_rand-x_near_cor).normalized();

        octomap::point3d x_new_cor=x_direction*STEPSIZE+x_near_cor;

        // geometry_msgs::Point point;
        // point.x=x_new_cor.x();
        // point.y=x_new_cor.y();
        // point.z=x_new_cor.z();
        // marker1.points.clear();
        // marker1.points.push_back(point);
        // std_msgs::ColorRGBA color;
        // color.r=1.0;color.g=1.0;color.b=0.0;color.a=1.0;
        // marker1.colors.clear();
        // marker1.colors.push_back(color);
        // marker.points.clear();
        // // marker_pub.publish(marker);
        // marker_pub.publish(marker1);

        
        ROS_INFO("x_new_cor: %f %f %f",x_new_cor.x(),x_new_cor.y(),x_new_cor.z());


        if (check_collision_free(x_near_cor,x_new_cor))
        {
            Node* x_new=new Node;
            x_new->setcoordinate(x_new_cor);
            x_new->setparentnode(node_list[x_near_index]);
            node_list.push_back(x_new);

            geometry_msgs::Point point;
            point.x=x_new_cor.x();
            point.y=x_new_cor.y();
            point.z=x_new_cor.z();

            marker.points.push_back(point);

            point.x=x_near_cor.x();
            point.y=x_near_cor.y();
            point.z=x_near_cor.z();
            marker.points.push_back(point);

            marker1.points.clear();

            point.x=node_list[0]->getcoordinate().x();point.y=node_list[0]->getcoordinate().y();point.z=node_list[0]->getcoordinate().z();

            marker1.colors.clear();
            std_msgs::ColorRGBA color;
            color.r=0.0;color.g=1.0;color.b=0.0;color.a=1.0;
            marker1.colors.push_back(color);
            color.r=1.0;color.g=0.0;color.b=0.0;color.a=1.0;
            marker1.colors.push_back(color);
            color.r=0.0;color.g=0.5;color.b=0.5;color.a=1.0;
            marker1.colors.push_back(color);

            marker1.points.push_back(point);  
            point.x=goal_point.x();point.y=goal_point.y();point.z=goal_point.z();
            marker1.points.push_back(point); 

            point.x=x_new_cor.x();point.y=x_new_cor.y();point.z=x_new_cor.z();
            marker1.points.push_back(point);  

            marker_pub.publish(marker);
            marker_pub.publish(marker1);


            if(x_new_cor.distance(goal_point)<0.3)
            {
                return true;
            }
            //ROS_INFO("x_new cor: %f %f %f",x_new_cor.x(),x_new_cor.y(),x_new_cor.z());
            //ROS_INFO("node_list.size(): %ld",node_list.size());
            //ROS_INFO("-----------------------");

        }
    }
    return false;
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

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);
    ros::Time last_gp_time_;
    ROS_INFO("rrt_star_planner start!!!!!!!!!");

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "rrt_star_planner"; 
    marker.id = 0;
    marker.type=visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.lifetime = ros::Duration();

    marker.color.r =1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.scale.x=0.02;

    marker1.header.frame_id = "/map";
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "rrt_star_planner"; 
    marker1.id = 1;
    marker1.type=visualization_msgs::Marker::SPHERE_LIST;
    marker1.action = visualization_msgs::Marker::ADD;

    marker1.lifetime = ros::Duration();

    marker1.color.r =0.0f;
    marker1.color.g = 1.0f;
    marker1.color.b = 0.0f;
    marker1.color.a = 1.0;
    marker1.scale.x=0.1;

    srand((unsigned)time(NULL));
    while(octree_==nullptr)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("octomap initialize success-----------");

    Node* x_start=new Node;
    octomap::point3d start_cor(0,0,1.5);
    x_start->setcoordinate(start_cor);
    x_start->parent_node=nullptr;
    node_list.push_back(x_start);

    while(ros::ok())
    {
        ros::spinOnce();

        if((ros::Time::now() - last_gp_time_).toSec()>0.003)
        {
            if(rrt_star_gp())
            {
                break;
            }
            last_gp_time_=ros::Time::now();
        }
    }
    ROS_WARN("motion planning complete,path points: ----------");
    Node* path_points=node_list[node_list.size()-1];
    std::vector<octomap::point3d> path_vector;
    while(path_points!=nullptr)
    {
        path_vector.insert(path_vector.begin(), path_points->getcoordinate());
        path_points->printcoordinate();
        path_points=path_points->getparentnode();
    }
    MatrixXd point=MatrixXd::Zero(path_vector.size(),3);
    for(int i=0;i<path_vector.size();i++)
    {
        point(i,0)=path_vector[i].x();
        point(i,1)=path_vector[i].y();
        point(i,2)=path_vector[i].z();
    }
    ROS_WARN("Start path smoothing------");
    MatrixXd plan_path_matrix= minimum_snap_plan(point);

    ROS_WARN("start tracking----------");
    // while(ros::ok())
    // {
    //     ros::spinOnce();
	//     loop_rate.sleep();
    // }
    return 0;
}


