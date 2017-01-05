//
// Created by tom on 20/04/16.
//

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


int main(int argc, char **argv) {
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("arm");

    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
    group.setStartStateToCurrentState();
    ROS_INFO("Planning reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="base_footprint";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(2.1);
    target_pose.pose.position.x = 0.5;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.9;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0); ;
    group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.move();
    }
    sleep(5);



    return 0;
}
