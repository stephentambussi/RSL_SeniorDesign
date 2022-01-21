#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;
void Joy_readings(const geometry_msgs::Twist& twist)
{
 ROS_INFO_STREAM("Subscriber velocities: "<<" linear= "<<twist.linear.x<<" angular="<<twist.angular.z);
}


int main(int argc, char **argv){
ros::init(argc, argv, "filter_velocity");
ros::NodeHandle nh;

ros::Subscriber sub = nh.subscribe("joy_test",1000,Joy_readings);
//ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/pose", 1000);

ros::Rate rate(2);

ros::spin();
}
