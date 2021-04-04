

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
using namespace Eigen;
using namespace std;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

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


	ROS_INFO("q: %f %f %f %f",q.x(),q.y(),q.z(),q.w());

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_link"));


}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_tf");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    const int LOOPRATE = 30;
    ros::Rate loop_rate(LOOPRATE);

    double yaw_set = 3.1415926;

    ROS_INFO("pc_tf start!!!!!!!!!");

    while(ros::ok())
    {
        ros::spinOnce();
    }
	loop_rate.sleep();


    return 0;
}
