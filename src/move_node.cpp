#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/SetBool.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

const double notInSightVal=-1;

const double pic_width=1000;
const double epsilon=5;
const double closeEnough=10;


const double searchVel=1;
const double searchAngle=0.17;
const double searchTime=10;

const double rotateVel=0;
const double rotateAngle=0.17;
const int rotateTime=10;

const double forwardVel=1;
const double forwardAngle=0;
const int forwardTime=10;



//Callback that is triggered after getting message of the red point location (relative to the robot)
//, moving the robot accordingly and then calling the service again to get the location over until the robot reaches the point
void getMessagesCallback(double arr[3]){
        //arr[0]=x-width, arr[1]=y-height, arr[2]=z-depth;
    
  
	//ros::init(argc, argv, "move_node");
	 ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        ros::Rate loop_rate(10);
	    
	if (arr[0]==notInSightVal && arr[1]==notInSightVal && arr[2]==notInSightVal) {
	     int count = 0;
	    while (ros::ok() && (forwardTime<=0 || count<forwardTime))
	    {
		  geometry_msgs::Twist cmd_msg;
		cmd_msg.linear.x = searchVel;
		cmd_msg.angular.z = searchAngle;

		chatter_pub.publish(cmd_msg);
		
		ros::spinOnce();
		
		 loop_rate.sleep();
		++count; 
	    }
	}
        if (arr[0]>(pic_width/2)-epsilon && arr[0]<(pic_width/2)+epsilon && arr[2]<=closeEnough) //reached point
	{
	  return;
	}
	else if (arr[0]>=(pic_width/2)-epsilon && arr[0]<=(pic_width/2)+epsilon){ //point in front of robot; moving forward
	       int count = 0;
	    while (ros::ok() && (forwardTime<=0 || count<forwardTime))
	    {

		  geometry_msgs::Twist cmd_msg;
		cmd_msg.linear.x = forwardVel;
		cmd_msg.angular.z = forwardAngle;

		chatter_pub.publish(cmd_msg);
		
		ros::spinOnce();
		
		 loop_rate.sleep();
		++count;
		
	    }
        }
        else if (arr[0]<pic_width/2) //point in sight and left of the robot -rotate left
        {
             int count = 0;
	    while (ros::ok() && (rotateTime<=0 || count<rotateTime))
	    {

		  geometry_msgs::Twist cmd_msg;
		cmd_msg.linear.x = rotateVel;
		cmd_msg.angular.z = rotateAngle;

		chatter_pub.publish(cmd_msg);
		
		ros::spinOnce();
		
		 loop_rate.sleep();
		++count;
		
	    }
        }
        else  if (arr[0]>pic_width/2)  //point in sight and right of the robot - rotate right
        {
           	      int count = 0;
	    while (ros::ok() && (rotateTime<=0 || count<rotateTime))
	    {

		  geometry_msgs::Twist cmd_msg;
		cmd_msg.linear.x = rotateVel;
		cmd_msg.angular.z = (-1)*rotateAngle;

		chatter_pub.publish(cmd_msg);
		
		ros::spinOnce();
		
		 loop_rate.sleep();
		++count;
  
	  }
        }
        
        //CALL YOGEV SERVICE
}

/*COMMAND TO RUN: move_node <Velocity> <Left Angle> <Time To Suspend>
DESCRIPTION TO THE THIRD PARAMETER <Time To Suspend>: The robot will stop moving after
<Time To Suspend>/10 seconds (I think). If this parameter is not inputted it will continue
moving until manually terminated.*/

int main(int argc, char **argv) {
	if (argc<3) return 0;

	ros::init(argc, argv, "move_node");
	
	ros::NodeHandle n;
ros::Publisher chatter_pub;
	chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Rate loop_rate(10);

	int count = 0;
	int stop_time=0;
	if (argc>3) stop_time=std::atoi(argv[3]);
	  while (ros::ok() && (stop_time<=0 || count<stop_time))
	  {

		  geometry_msgs::Twist cmd_msg;
		cmd_msg.linear.x = std::atof(argv[1]);
		cmd_msg.angular.z = std::atof(argv[2]);

		chatter_pub.publish(cmd_msg);
		
		ros::spinOnce();
		
		 loop_rate.sleep();
		++count;
  
	  }

return 0;



}
