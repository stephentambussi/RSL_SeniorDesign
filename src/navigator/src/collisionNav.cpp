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
float maxDistance = .25;

float degreeMemory = 0;

ros::Publisher p; //publisher for linang array
std_msgs::Float32MultiArray linang;

float objectVector [2]; //index 0 is x, index 1 is y

void movements(float degree, float distance){
    objectVector[0] = 0;
    objectVector[1] = 0;

    if(degree < -110 || degree > 110){
        if(distance < 1.25){
            float objectYPlane = distance*sin(degree-180);
            if(abs(objectYPlane) < minimalSafeY){
                
                degreeMemory = degree;

                if(degree < -100)
                    objectVector[1] = (1/distance);
                else
                    objectVector[1] = -(1/distance);
                objectVector[0] = -(1/distance);

                if(objectVector[0] < -0.5) objectVector[0] = -0.5; //x speed cap for demo
                if(objectVector[1] > 0.5) objectVector[1] = 0.5; //y speed cap for demo
                else if(objectVector[1] < -0.5) objectVector[1] = -0.5; //y speed cap for demo
                //ROS_INFO("Distance: %f \t Degree: %f \t xVector,yVector %f, %f", distance, degree, objectVector[0], objectVector[1]);

                linang.data[0] = objectVector[0];
                linang.data[1] = objectVector[1]; 
           }

        }else{
            float degreePos = 180 - degree;
            float degreePosMemory = 180 - degreeMemory;
            if(degreePos > degreePosMemory - 5 && degreePos < degreePosMemory + 5)
            {
                linang.data[0] = .3;
                linang.data[1] = 0;
                linang.data[2] = 0;
            }
            
        }
    }
  
    /*
    degree = 180 - degree;
    float objectYPlane = 0;
    float objectXPlane = 0;
    if(degree == 0)
    {
        //objectYPlane = 0, only objectXPlane
        objectYPlane = distance*sin(degree);
        objectXPlane = distalinang.data.push_back(1); //x
    linang.data.push_back(0); //y
    linang.data.push_back(0); //z
    if(degree > 270  || degree < 90){
        objectYPlane = distance*sin(degree);
        objectXPlane = distance*cos(degree);
        if(abs(objectYPlane) < minimalSafeY && distance < 1.5){
            if(degree > 270){  //on the left side
                ROS_INFO("On the left side and in collision range of distance %f and degree %f", distance,degree);
                //ROS_INFO("X distance from object: %f", objectXPlane);
            }
            if(degree < 90){
                ROS_INFO("On the right side and in collision range of distance %f and degree %f", distance,degree);
                 ROS_INFO("X distance from object: %f", objectXPlane);
            }
        }
    }
    */
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

        p.publish(linang);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    //initialize array
    linang.data.push_back(0); //x
    linang.data.push_back(0); //y
    linang.data.push_back(0); //z

    linang.data[0] = .3;
    linang.data[1] = 0;
    linang.data[2] = 0;

    p = n.advertise<std_msgs::Float32MultiArray>("zed_vel1", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}