//
// Created by tom on 26/06/16.
//

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

bool gripper_cmd(GripperClient* gripperClient,double gap,double effort) {

    control_msgs::GripperCommandGoal openGoal;

    openGoal.command.position = gap;
    openGoal.command.max_effort = effort;
    gripperClient->sendGoal(openGoal);
    ROS_INFO("Sent gripper goal");
    gripperClient->waitForResult();

    if(gripperClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper done");
        return true;
    }
    else {
        ROS_ERROR("Gripper fault");
        // return false;
    }
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_api");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;


    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    //wait for the gripper action server to come up
    while (!gripperClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the /gripper_controller/gripper_cmd action server to come up");
    }

    gripper_cmd(&gripperClient,0.14,0);
    sleep(5);
    gripper_cmd(&gripperClient,0.01,0.2);

}
