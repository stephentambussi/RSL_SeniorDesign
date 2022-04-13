#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//Initial Pose
double x = 0.0;
double y = 0.0;
double th = 0.0;

//Initial velocities
double vx = 0.0; //linear x velocity - m/s
double vy = 0.0; //linear y velocity - m/s
double vth = 0.0; //angular velocity - rad/s

//Robot physical constants
//const double TICKS_PER_REVOLUTION = 854; // For reference purposes.
const double WHEEL_RADIUS = 0.07; // Wheel radius in meters
const double WHEEL_BASE = 0.42; // Center of left tire to center of right tire
const double circum = 2 * M_PI * WHEEL_RADIUS; //get circumference of wheel (distance traveled per 1 revolution)

//Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;

//TODO: get RPMs from all 4 wheels, convert to rad/s, calculate velocity

void calc_vel()
{
    //TODO: calculate
}

//Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {

  x = rvizClick.pose.position.x;
  y = rvizClick.pose.position.y;
  th = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}

//Update the odometry information
void update_odom()
{
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    //Calculate pose
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    /* -- may be unnecessary -- see if it works without it
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    */

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
    ros::Subscriber subInitialPose = n.subscribe("initial_2d", 1, set_initial_2d);
    //TODO: change these subscribers when you change serial publisher for RPMs
    ros::Subscriber subForRightCounts = n.subscribe("right_RPMs", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = n.subscribe("left_RPMs", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
    
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(30); //30 times per second

    while(ros::ok())
    {
        if(initialPoseReceived)
        {
            calc_vel();
            update_odom();
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}