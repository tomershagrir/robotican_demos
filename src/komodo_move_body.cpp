


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <robotican_common/FindObjectDynParamConfig.h>
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <robotican_common/switch_topic.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "robotican_demos/arm_msg.h"

using namespace cv;

bool debug_vision=false;

void rotateBody(int rotateTime, double rotateVel, double rotateAngle);

//bool timeout=true;

int object_id;

//ros::Time detect_t;

//red
int minH=3,maxH=160;
int minS=70,maxS=255;

int minV=10,maxV=255;
int minA=200,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

int inv_H=1;


const double notInSightVal=-1;

const double pic_width=0.4;
const double epsilon=0.1;
const double closeEnough=0.62;


const double searchVel=1;
const double searchAngle=0.17;
const double searchTime=10;

const double rotateVel=0;
const double rotateAngle=0.5;
const int rotateTime=10;

const double forwardVel=0.2;
const double forwardAngle=0;
const int forwardTime=10;

int focus=0;

int main(int argc, char **argv) {

    ros::init(argc, argv, "find_objects_node");
    ros::NodeHandle n;
     ros::NodeHandle pn("~");
    ROS_INFO("Hello");

   // std::string object_id;
  //  pn.param<double>("object_r", object_r, 0.025);
  //  pn.param<double>("object_h", object_h, 0.15);
   // pn.param<std::string>("object_id", object_id, "can");

    pn.param<int>("object_id", object_id, 1);

    pn.param<std::string>("depth_topic1", depth_topic1, "/kinect2/qhd/points");
    pn.param<std::string>("depth_topic2", depth_topic2, "/sr300/depth/points");
    depth_topic=depth_topic1;

    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig> dynamicServer;
    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig>::CallbackType callbackFunction;

    callbackFunction = boost::bind(&dynamicParamCallback, _1, _2);
    dynamicServer.setCallback(callbackFunction);

    image_transport::ImageTransport it_(pn);

    result_image_pub = it_.advertise("result", 1);
    object_image_pub = it_.advertise("hsv_filterd", 1);
    bw_image_pub = it_.advertise("bw", 1);
    ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
    ROS_INFO_STREAM(depth_topic);


    object_pub=n.advertise<ar_track_alvar_msgs::AlvarMarkers>("detected_objects", 2, true);

    pose_pub=pn.advertise<geometry_msgs::PoseStamped>("object_pose",10);


    tf::TransformListener listener;
    listener_ptr=&listener;

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(pn, "object_pose", 10);

    tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
    tf_filter.registerCallback( boost::bind(obj_msgCallback, _1) );


ros::ServiceServer switch_sub = n.advertiseService("switch_pcl_topic", &switch_pcl_topic);

ros::Rate r(10);
    ROS_INFO("Ready to find objects!");
    while (ros::ok()) {
    if (pcl_sub.getTopic()!=depth_topic) {
        pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
         ROS_INFO("switching pcl topic");
    }
     ros::spinOnce();
     r.sleep();
    }

    return 0;
}


void rotateBody(int rotateTime, double rotateVel, double rotateAngle)
{
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Rate loop_rate(10);
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
  ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
}