#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32.h"
#include <sensor_msgs/MagneticField.h>

ros::Publisher p;
std_msgs::Float32 orientation;

using namespace std;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    /*
  ROS_INFO("Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
           msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x,
           msg->angular_velocity.y, msg->angular_velocity.z, msg->orientation.x, msg->orientation.y, msg->orientation.z,
           msg->orientation.w);
    */
   //cout << "Orientation around z-axis: " << msg->orientation.z << endl;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
  ROS_INFO("Mag. Field: %.3f,%.3f,%.3f [uT]", msg->magnetic_field.x, msg->magnetic_field.y,
           msg->magnetic_field.z);
}

int main(int argc, char** argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "imu_heading");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called imageCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //ros::Subscriber subImu = n.subscribe("/zed2i/zed_node/imu/data", 10, imuCallback);
  //ros::Subscriber subImu = n.subscribe("heading", 10, imuCallback);
  ros::Subscriber subMag = n.subscribe("/zed2i/zed_node/imu/mag", 10, magCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
