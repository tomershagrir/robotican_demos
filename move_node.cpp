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

int main(int argc, char **argv) {

	ros::init(argc, argv, "move_node");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Rate loop_rate(10);

	int count = 0;
	  while (ros::ok())
	  {

		  geometry_msgs::Twist cmd_msg;
		cmd_msg.linear.x = 0.5;
		cmd_msg.angular.z = 0;

		chatter_pub.publish(cmd_msg);
		
		ros::spinOnce();
		
		 loop_rate.sleep();
		++count;
  
	  }

return 0;



}
