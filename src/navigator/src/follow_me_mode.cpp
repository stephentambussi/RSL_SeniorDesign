#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>

#include <zed_interfaces/Object.h>
#include <zed_interfaces/ObjectsStamped.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"

ros::Publisher p; //publisher for linang array
std_msgs::Float32MultiArray linang;

float move_xthreshold = 1.22; //in meters - for robot to move forward
float rotate_xthreshold = 0.5; //in meters - for robot to rotate inside of strafe
float linear_ythreshold = 0.2; //in meters - for robot to strafe or rotate
float linear_x = 0;
float linear_y = 0;
float angular_z = 0;
int tracking_id = 0;
int init_tracking_flag = 0;
int timer_called_flag = 0;
auto start = std::chrono::high_resolution_clock::now(); //timer check
auto finish = std::chrono::high_resolution_clock::now();

int collision_flag = 0;

void collisionCallback(const std_msgs::Int16& signal)
{
  if(signal.data == 616)
  {
    collision_flag = 1; //collision detected
  }
}

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */
void objectListCallback(const zed_interfaces::ObjectsStamped::ConstPtr& msg)
{
  int flag = 0;
  float max_confid = 65;
  int max_confid_id = 0;
  if(init_tracking_flag == 0) //initialize object det with the person with highest confidence
  {
    for(int i = 0; i < msg->objects.size(); i++)
    {
      if(msg->objects[i].confidence > max_confid)
      {
        max_confid = msg->objects[i].confidence;
        tracking_id = msg->objects[i].label_id;
      }
    }
    init_tracking_flag = 1;
    max_confid = 65;
  }
  ROS_INFO("***** New object list *****");
  for (int i = 0; i < msg->objects.size(); i++)
  {
    if (msg->objects[i].label_id == -1)
      continue;

    // Output object type, object id, object current position vector, object confidence, and object tracking state
    // Tracking state values: 0 - OFF (object not tracked) | 1 - OK (object tracked) | 2 - SEARCHING (trajectory estimated)
    ROS_INFO_STREAM(msg->objects[i].label << " [" << msg->objects[i].label_id << "] - Pos. ["
                                          << msg->objects[i].position[0] << "," << msg->objects[i].position[1] << ","
                                          << msg->objects[i].position[2] << "] [m]"
                                          << " - Vel. ["
                                          << msg->objects[i].velocity[0] << "," << msg->objects[i].velocity[1] << ","
                                          << msg->objects[i].velocity[2] << "] [m/s]" 
                                          << "- Conf. " << msg->objects[i].confidence
                                          << " - Tracking state: " << static_cast<int>(msg->objects[i].tracking_state));

    //calculate linear and angular vel to send to driver_node
    //TODO1: Code so that robot only follows single person at a time, even with multiple people in view
    //TODO2: Code so that robot matches velocity of person it is actively following (including angular)
    if(msg->objects[i].confidence > max_confid) //keep track of obj with highest confidence in case tracked obj not found
    {
      max_confid = msg->objects[i].confidence;
      max_confid_id = msg->objects[i].label_id;
    }
    if(msg->objects[i].label_id == tracking_id && static_cast<int>(msg->objects[i].tracking_state) == 1
     && collision_flag == 0) //if object is tracked one and actively being tracked
    {
      linang.data.clear();
      ROS_INFO_STREAM("Tracking ID: " << tracking_id);
      if(msg->objects[i].position[0] > move_xthreshold)
      {
        linear_x = 0.4; //move forward in x direction
      }
      if(msg->objects[i].position[0] <= move_xthreshold)
      {
        linear_x = 0; //stop moving
      }
      if(abs(msg->objects[i].position[1]) > linear_ythreshold)
      {
        if(msg->objects[i].position[1] < 0)
        {
          angular_z = -0.3; //rotate robot right
        }
        else
        {
          angular_z = 0.3; //rotate robot left
        }
      }
      if(abs(msg->objects[i].position[1]) <= linear_ythreshold)
      {
        angular_z = 0; //stop rotating
        linear_y = 0; //stop strafing
      }
      linang.data.push_back(linear_x);
      linang.data.push_back(linear_y);
      linang.data.push_back(angular_z);
  
      p.publish(linang); //send linear and angular velocities to driver_node
      flag = 1;
      break;
    }
  }
  if(collision_flag == 1) //Completely stop robot from moving on collision detection
  {
    linear_x = 0;
    angular_z = 0;
    linang.data.push_back(linear_x);
    linang.data.push_back(linear_y);
    linang.data.push_back(angular_z);

    p.publish(linang);
  }
  if(flag == 0) //if tracked object not detected, start change object timer
  {
    linang.data.clear();
    linang.data.push_back(0); //stop robot when it loses tracked object
    linang.data.push_back(0);
    linang.data.push_back(0);
    p.publish(linang); //send linear and angular velocities to driver_node
    if(timer_called_flag == 0)
    {
      //start timer countdown -- call function
      start = std::chrono::high_resolution_clock::now();
      timer_called_flag = 1;
    }
    else if(timer_called_flag == 1){ //when object is still undetected
      finish = std::chrono::high_resolution_clock::now();
      auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish-start);
      if(microseconds >= std::chrono::seconds(3))
      {
        timer_called_flag = 0;
        tracking_id = max_confid_id; //change to new object
      }
    }
  }
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
  ros::init(argc, argv, "obj_det");

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
  p = n.advertise<std_msgs::Float32MultiArray>("zed_vel1", 10);
  ros::Subscriber subObjList = n.subscribe("objects", 10, objectListCallback);
  ros::Subscriber subCollSignal = n.subscribe("collisionSignal", 10, collisionCallback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
