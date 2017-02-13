typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher valPub;
geometry_msgs::Vector3 replay;
 
void Callback(const PointCloud::ConstPtr& msg) {
    // set initial min distance to infinity
    float x,y,z;
    x = y = z = FLT_MAX;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        // ignore nan values
        if( !pcl_isfinite( pt.x ) || !pcl_isfinite( pt.y ) || !pcl_isfinite( pt.z ) ) continue;
        // update min distance when we reache a new minimum z
        if (pt.z < z) { 
           x = pt.x;
           y = pt.y;
           z = pt.z;
       }
    }
    replay.x = x;
    replay.y = y;
    replay.z = z;
    valPub.publish(replay);
}
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_min");
    ros::NodeHandle n;
    // Tell ROS we want to publish velocity message
    valPub = n.advertise<geometry_msgs::Vector3>("cloud_minRange",10);
    // Listen to point cloud messages
    ros::Subscriber sub = n.subscribe<PointCloud>("komodo_1/komodo_1_Asus_Camera/depth/points", 1,Callback);
    ros::spin(); // run callbacks (blocking)
}
