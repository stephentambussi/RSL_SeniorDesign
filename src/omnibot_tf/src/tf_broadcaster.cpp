#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;
    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;
    while(n.ok()){
        broadcaster.sendTransform(
            tf::StampedTransform(
                //tf::Vector3(x, y, z) in meters will change depending on physical location of sensor on robot
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.28, 0.0, 0.1)),
                ros::Time::now(),"base_link", "base_laser"));
        r.sleep();
    }
}