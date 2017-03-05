


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

//#include "robotican_demos/arm_msg.h"

using namespace cv;

bool debug_vision=false;


bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level);
void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);

void rotateBody(int rotateTime, double rotateVel, double rotateAngle);


ros::Publisher chatter_pub;

tf::TransformListener *listener_ptr;

//bool timeout=true;

int object_id;

//ros::Time detect_t;

std::string depth_topic1,depth_topic2,depth_topic;
bool have_object=false;

ros::Publisher object_pub;
image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;
ros::Publisher pose_pub;
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



void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    ROS_INFO("Entered Cloud Call Back");
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {

        ROS_WARN("empty cloud");
        return;
    }

    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    Mat result=cv_ptr->image;

    Point3d obj;
    have_object= find_object(result,cloudp,&obj,input->header.frame_id);

    waitKey(1);

    

    if (have_object) 
    {
      ROS_INFO("Object has been found!!!");
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id=input->header.frame_id;
      target_pose.header.stamp=ros::Time::now();
      target_pose.pose.position.x =obj.x;
      target_pose.pose.position.y = obj.y;
      target_pose.pose.position.z = obj.z;


      target_pose.pose.orientation.w=1;
      //pose_pub.publish(target_pose);
	
      double arr [3] = { obj.x, obj.y, obj.z };
      //arr[0]=x-width, arr[1]=y-height, arr[2]=z-depth;
    
  
	    //ros::init(argc, argv, "move_node");
	    ros::NodeHandle n;
      ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
      ros::Rate loop_rate(10);
	    
	    if (arr[0]==notInSightVal && arr[1]==notInSightVal && arr[2]==notInSightVal) 
      {
		  
        //rotateBody(forwardTime, searchVel, searchAngle);
	    }
      if (((arr[0]>=(pic_width/2)-epsilon && arr[0]<=(pic_width/2)+epsilon) || focus) && arr[2]<=closeEnough) 
    	{
        //can reach point with arm
		ROS_INFO("move arm!!");
        static const std::string ARM_PLANNING_GROUP = "/torso_position_controller/command";
        moveit::planning_interface::MoveGroup move_group(ARM_PLANNING_GROUP);

        geometry_msgs::Pose target_pose;
        target_pose.position.x = arr[0];
        target_pose.position.y = arr[1];
        target_pose.position.z = arr[2];
        move_group.setPoseTarget(target_pose);

        ROS_INFO("moving arm");

        move_group.move();

/*
        robotican_demos::arm_msg msg;
        msg.x = arr[0];
        msg.y = arr[1];
        msg.z = arr[2];

        ros::NodeHandle arm_node_handle;
        ros::Publisher arm_chatter_pub = arm_node_handle.advertise<robotican_demos::arm_msg>("chatter", 1000);
        int count = 0;
        int stop_time = 10;
        while (ros::ok() && (stop_time<=0 || count<stop_time))
        {
          arm_chatter_pub.publish(msg);

          ros::spinOnce();

          loop_rate.sleep();
          ++count;
        }*/
    	}
	    else if ((arr[0]>=(pic_width/2)-epsilon && arr[0]<=(pic_width/2)+epsilon) || focus)
      { //point in front of robot; moving forward
	      focus=1;
	      
	      ROS_INFO("front!!!!");
        //rotateBody(forwardTime, forwardVel, forwardAngle);
        
        ROS_INFO("move arm!!");
        static const std::string ARM_PLANNING_GROUP = "arm";
        moveit::planning_interface::MoveGroup move_group(ARM_PLANNING_GROUP);

        geometry_msgs::Pose target_pose;
        target_pose.position.x = arr[0];
        target_pose.position.y = arr[1];
        target_pose.position.z = arr[2];
        move_group.setPoseTarget(target_pose);

        ROS_INFO("moving arm");

        move_group.move();
      }
      else if (arr[0]<pic_width/2) 
      {//point in sight and left of the robot -rotate left
        //rotateBody(rotateTime, rotateVel, rotateAngle);
        ROS_INFO("left!!!");
      }
      else  if (arr[0]>pic_width/2)  
      {//point in sight and right of the robot - rotate right
        //rotateBody(rotateTime, rotateVel, (-1)*rotateAngle);
        ROS_INFO("right!!!");
      }
    }
    else
    {
      ros::NodeHandle n;
      ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
      ros::Rate loop_rate(10);
      //rotateBody(rotateTime, rotateVel, (-1)*rotateAngle);
    }
}

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr,std::string frame) 
{

  Mat hsv,filtered,bw,mask;

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp=ros::Time::now();
  out_msg.header.frame_id=  frame;

  cvtColor(input,hsv,CV_BGR2HSV);


  if (inv_H) 
  {
      Mat lower_hue_range;
      Mat upper_hue_range;
      inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
      inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);
      // Combine the above two images

      addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
  }
  else{
      //if not red use:
      inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),mask);
  }
  hsv.copyTo(filtered,mask);
  cvtColor(filtered,filtered,CV_HSV2BGR);

  out_msg.image    = filtered;
  out_msg.encoding = "bgr8";
  object_image_pub.publish(out_msg.toImageMsg());

  mask.copyTo(bw);
  if (gaussian_ksize>0) {
      if (gaussian_ksize % 2 == 0) gaussian_ksize++;
      GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
  }


  if (morph_size>0) {
      Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
      morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
  }



  out_msg.image    = bw;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  bw_image_pub.publish(out_msg.toImageMsg());


  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  double largest_area=0;
  int largest_contour_index=0;
  for( int i = 0; i< contours.size(); i++ )
  {
      double area0 = abs(contourArea(contours[i]));
      if(area0>largest_area)
      {
          largest_area=area0;
          largest_contour_index=i;
      }
  }
  bool ok=false;
  if ((largest_area>minA)&&(largest_area<maxA)) 
  {
    drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
    Moments mu=moments( contours[largest_contour_index], true );
    Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );
    int pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
    circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

    pr->x=cloudp->points[pcl_index].x;
    pr->y=cloudp->points[pcl_index].y;
    pr->z=cloudp->points[pcl_index].z;
    char str[100];
    if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
        sprintf(str,"NaN");
        ok=false;
    }
    else {
        sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
        ok=true;
    }
    putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(255,255,255), 1, 8);
  }
  out_msg.image    = input;
  out_msg.encoding = "bgr8";
  result_image_pub.publish(out_msg.toImageMsg());

  return ok;
}

int main(int argc, char **argv) 
{
	ROS_INFO("start");
  ros::init(argc, argv, "find_objects_node");
  ros::NodeHandle n1;

  pose_pub=n1.advertise<geometry_msgs::PoseStamped>("object_pose",10);
  chatter_pub = n1.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber pcl_sub = n1.subscribe("/torso_camera/depth_registered/points", 1, cloud_cb);
  
  ros::spin();
  
  return 0;
}

/*
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
}*/
