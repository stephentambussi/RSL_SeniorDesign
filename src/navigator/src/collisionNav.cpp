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
float minmialSafeX = 0.4572;

ros::Publisher p; //publisher for linang array
std_msgs::Float32MultiArray linang;

void movements(float degree, float distance){
    degree = 180 - degree;
    float objectYPlane = 0;
    float objectXPlane = 0;
    if(degree == 0)
    {
        //objectYPlane = 0, only objectXPlane
        objectYPlane = distance*sin(degree);
        objectXPlane = distance*cos(degree);
        //ROS_INFO("X distance from object: %f", objectXPlane);

    }
    if(degree > 270  || degree < 90){
        objectYPlane = distance*sin(degree);
        objectXPlane = distance*cos(degree);
        if(abs(objectYPlane) < minimalSafeY && distance < 1.5){
            if(degree > 270){  //on the left side
                ROS_INFO("On the left side and in collision range of distance %f and degree %f", distance,degree);
                ROS_INFO("X distance from object: %f", objectXPlane);
            }
            if(degree < 90){
                ROS_INFO("On the right side and in collision range of distance %f and degree %f", distance,degree);
                 ROS_INFO("X distance from object: %f", objectXPlane);
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