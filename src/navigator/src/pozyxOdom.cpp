#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//Initial Pose
double x = 0.0;
double y = 0.0;
double th = 0.0;

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");

    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
    ros::Subscriber subInitialPose = n.subscribe("initial_2d", 1, set_initial_2d);
    ros::Subscriber subForEnc1Counts = n.subscribe("enc1_RPMs", 100, enc1_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForEnc2Counts = n.subscribe("enc2_RPMs", 100, enc2_callback, ros::TransportHints().tcpNoDelay());

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate r(30); //15 times per second

    while(ros::ok())
    {
        ros::spinOnce();
        if(initialPoseReceived)
        {
            update_odom();
        }
        r.sleep();
    }
    return 0;
}