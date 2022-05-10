#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//Initial Pose
double x = 0.0;
double y = 0.0;
double th = 0.0;

ros::Time current_time, last_time;
ros::Publisher odom_pub;

using namespace std;

void pozyx_callback(const std_msgs::Float32MultiArray& coords)
{
    tf::TransformBroadcaster odom_broadcaster;
    current_time = ros::Time::now();

    x = coords.data[0];
    y = coords.data[1];

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); //TODO: see if you should change this or not

    //publish transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint"; //TODO: check transform

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send transform -- TEST
    //odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom"; //TODO: check this
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,  // covariance on gps_x
                            0, 0.01, 0, 0, 0, 0,  // covariance on gps_y
                            0, 0, 0.01, 0, 0, 0,  // covariance on gps_z
                            0, 0, 0, 99999, 0, 0,  // large covariance on rot x
                            0, 0, 0, 0, 99999, 0,  // large covariance on rot y
                            0, 0, 0, 0, 0, 99999};  // large covariance on rot z
    odom_pub.publish(odom);
    last_time = current_time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pozyx_gps");

    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("pozyx_gps_data", 100);
    ros::Subscriber subForPozyx = n.subscribe("coordinates", 100, pozyx_callback);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::spin();

    return 0;
}