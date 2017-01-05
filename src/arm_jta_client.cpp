
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/client/action_client.h>

bool debug=true;


int main(int argc, char **argv) {
    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle n;

    moveit::planning_interface::MoveGroup group("arm");

    actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> ac(n, "/arm_trajectory_controller/follow_joint_trajectory");

    std::vector< double > current_vals = group.getCurrentJointValues();
    std::vector< std::string > joint_names = group.getJoints();
    joint_names.resize(6);

    std::vector< double > vals;
    vals.resize(6);


    for (int i=0;i<vals.size();i++) {
        vals[i]=0;
    }


    for (int i=0;i<current_vals.size();i++) {

        std::cout << joint_names[i] <<"   "<<current_vals[i] << std::endl;
    }


    std::srand(std::time(0));

    control_msgs::FollowJointTrajectoryActionGoal goal;
    goal.header.stamp = ros::Time::now();

    goal.goal_id.stamp = ros::Time::now();
    std::ostringstream oss;
    oss << "goal_" << std::rand();
    goal.goal_id.id = oss.str();

    goal.goal.trajectory.joint_names=joint_names;
    goal.goal.trajectory.header.frame_id="/map";

    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.time_from_start = ros::Duration(5.0);
    jtp.positions=vals;
    jtp.velocities.resize(6);
    jtp.accelerations.resize(6);

    goal.goal.trajectory.points.push_back(jtp);

    for (int i=0;i<vals.size();i++) {
        vals[i]=-0.2;
    }
    jtp.time_from_start = ros::Duration(10.0);
    jtp.positions=vals;
    goal.goal.trajectory.points.push_back(jtp);

    ac.sendGoal(goal.goal);



    if (debug) {
        std::stringstream s;
        std::string indent("*> ");
        ros::message_operations::Printer<control_msgs::FollowJointTrajectoryActionGoal> p;
        p.stream(s, indent, goal);
        std::cout << s.str() << std::endl;
    }


    ros::spin();

    return 0;
}
