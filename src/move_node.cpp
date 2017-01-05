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


/*COMMAND TO RUN: move_node <Velocity> <Left Angle> <Time To Suspend>
DESCRIPTION TO THE THIRD PARAMETER <Time To Suspend>: The robot will stop moving after
<Time To Suspend>/10 seconds (I think). If this parameter is not inputted it will continue
moving until manually terminated.*/

int main(int argc, char **argv) {
	if (argc<3) return 0;

	ros::init(argc, argv, "move_node");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
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
