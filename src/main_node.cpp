#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotican_demos/arm_msg.h"


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




#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */


  ros::NodeHandle body_node_handle;
  ros::Publisher chatter_pub = body_node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Rate loop_rate(10);

  int count = 0;
  int stop_time=10;
  bool foundLightSwitch = false;

  while (!foundLightSwitch)
  {
    while (ros::ok() && (stop_time<=0 || count<stop_time))
    {

      geometry_msgs::Twist cmd_msg;
      cmd_msg.linear.x = 0.6;
      cmd_msg.angular.z = -0.9;

      chatter_pub.publish(cmd_msg);
      
      ros::spinOnce();
      
      loop_rate.sleep();
      ++count;

    }
    //check with pointcloud if lightswitch is found
  }

  //move towards light switch









  //move arm towards light switch
  ros::NodeHandle arm_node_handle;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher arm_chatter_pub = arm_node_handle.advertise<robotican_demos::arm_msg>("chatter", 1000);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  count = 0;
  while (ros::ok() && (stop_time<=0 || count<stop_time))
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    robotican_demos::arm_msg msg;

    //msg.w = 0.1;
    msg.x = 0.1;
    msg.y = 0.1;
    msg.z = 0.1;



    /*std::stringstream ss;
    Get real parameters from tutorial
    ss << "1 2 3 4" << count;
    msg.data = ss.str();*/

    ROS_INFO("%f, %f, %f", msg.x, msg.y, msg.z);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    arm_chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}