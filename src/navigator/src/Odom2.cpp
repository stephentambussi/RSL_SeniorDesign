#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//Initial Pose
double x = 0.0;
double y = 0.0;
double th = 0.0;

//Robot physical constants
//const double TICKS_PER_REVOLUTION = 854; // For reference purposes.
const double WHEEL_RADIUS = 0.07; // Wheel radius in meters
const double wheel_to_center_x = 0.21; // center of tire to center of robot
const double wheel_to_center_y = 0.1715;
const double circum = 2 * M_PI * WHEEL_RADIUS; //get circumference of wheel (distance traveled per 1 revolution)

//RPM to rad/s conversion constant
const double rpm_to_rad = 0.10472;

//Flag to see if initial pose has been received
bool initialPoseReceived = false;

ros::Time current_time, last_time;
ros::Publisher odom_pub;

int print_delay = 0;

//              {m1, m2, m3, m4}
int rpms [4] = {0, 0, 0, 0}; //initialize empty rpm array

using namespace std;

void enc1_callback(const std_msgs::Int32MultiArray& enc1_RPMs)
{
    rpms[0] = int(enc1_RPMs.data[0]);
    rpms[1] = int(enc1_RPMs.data[1]);
}

void enc2_callback(const std_msgs::Int32MultiArray& enc2_RPMs)
{
    rpms[2] = int(enc2_RPMs.data[0]);
    rpms[3] = int(enc2_RPMs.data[1]);
}

//Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {

  x = rvizClick.pose.position.x;
  y = rvizClick.pose.position.y;
  th = rvizClick.pose.orientation.z;
  initialPoseReceived = true;
}

//Update the odometry information
void update_odom()
{
    //TODO: odom does not work because asynchronous nature of above callbacks, need to synchronize somehow
    current_time = ros::Time::now();

    //Initial velocities
    double vx = 0.0; //linear x velocity - m/s
    double vy = 0.0; //linear y velocity - m/s
    double vth = 0.0; //angular velocity - rad/s

    //Calculate velocity
    double w1 = rpms[0] * rpm_to_rad; //in rad/s
    double w2 = rpms[1] * rpm_to_rad;
    double w3 = rpms[2] * rpm_to_rad;
    double w4 = rpms[3] * rpm_to_rad;
    vx = (w1 + w2 + w3 + w4) * (WHEEL_RADIUS / 4);
    if(vx < 0.05 && vx > -0.05)
        vx = 0;
    vy = (w2 + w3 - w1 - w4) * (WHEEL_RADIUS / 4);
    if(vy < 0.05 && vy > -0.05)
        vy = 0;
    double wheel_to_center_sum = wheel_to_center_x + wheel_to_center_y;
    vth = (w2 + w4 - w1 - w3) * (WHEEL_RADIUS / (4 * wheel_to_center_sum));
    if(vth < 0.05 && vth > -0.05)
        vth = 0;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    //Calculate pose
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
    if(print_delay % 30 == 0)
    {
        cout << "Positions: x = " << x << " y = " << y << " theta = " << th << endl;
        cout << "Velocities: x = " << vx << " y = " << vy << " theta = " << vth << endl;
        print_delay = 0;
    }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

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
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = vth;

    //Using small dummy values for the covariance matrix
    for(int i = 0; i<36; i++) {
        if(i == 0 || i == 7 || i == 14) {
            odom.pose.covariance[i] = .01;
        }
        else if (i == 21 || i == 28 || i== 35) {
            odom.pose.covariance[i] += 0.1;
        }
        else {
            odom.pose.covariance[i] = 0;
        }
    }

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    print_delay++;
}

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