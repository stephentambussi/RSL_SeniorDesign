#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

float minimalSafeY=.4572;  //18 inches in meters 

ros::Publisher p; //publisher for linang array
std_msgs::Float32MultiArray linang;

void movements(float degree, float distance){
    degree = 180 - degree;
    if(degree > 270  || degree < 90){
        float objectYPlane = distance*sin(degree);
        if(abs(objectYPlane) < minimalSafeY && distance < 1.5){
            if(degree > 270){  //on the left side
                ROS_INFO("On the left side and in collision range of distance %f and degree %f", distance,degree);
            }
            if(degree < 90){
                ROS_INFO("On the right side and in collision range of distance %f and degree %f", distance,degree);
            }
        }
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        movements(degree, scan->ranges[i]);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    p = n.advertise<std_msgs::Float32MultiArray>("zed_vel2", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}