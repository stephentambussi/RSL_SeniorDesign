#include "ros/ros.h" 
#include "std_msgs/Int32MultiArray.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <chrono>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
//const double TICKS_PER_REVOLUTION = 854; // For reference purposes.
const double WHEEL_RADIUS = 0.07; // Wheel radius in meters
const double WHEEL_BASE = 0.42; // Center of left tire to center of right tire
const double circum = 2 * M_PI * WHEEL_RADIUS; //get circumference of wheel (distance traveled per 1 revolution)

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;

// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {

  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int32MultiArray& leftRPMs) {
  //cout << "Left motor RPMs: " << leftRPMs.data[0] << endl;
  static chrono::steady_clock::time_point startL;
  static int start_flagL = 0;
  if(leftRPMs.data[0] != 0 && start_flagL != 0)
  {
    auto endL = chrono::steady_clock::now();
    auto durL = chrono::duration<double>(endL-startL).count();
    //cout << "Duration: " << durL << endl;
    double dist_per_min = circum * leftRPMs.data[0]; //calculate the distance traveled per minute from current RPM value
    //calculate the distance traveled since the last cycle
    distanceLeft = dist_per_min * (durL / 60.0);
    //cout << "Distance since last cycle: " << distanceLeft << endl;
  }
  startL = chrono::steady_clock::now();
  start_flagL = 1;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int32MultiArray& rightRPMs) {
  //cout << "Right motor RPMs: " << rightRPMs.data[0] << endl;
  static chrono::steady_clock::time_point startR;
  static int start_flagR = 0;
  if(rightRPMs.data[0] != 0 && start_flagR != 0)
  {
    auto endR = chrono::steady_clock::now();
    auto durR = chrono::duration<double>(endR-startR).count();
    //cout << "Duration: " << durR << endl;
    double dist_per_min = circum * rightRPMs.data[0]; //calculate the distance traveled per minute from current RPM value
    //calculate the distance traveled since the last cycle
    distanceRight = dist_per_min * (durR / 60.0);
    //cout << "Distance since last cycle: " << distanceRight << endl;
  }
  startR = chrono::steady_clock::now();
  start_flagR = 1;
}

// Publish a nav_msgs::Odometry message in quaternion format
// I will use small, dummy values for the covariance matrix. These values 
// are sometimes provided in the datasheet for the IMU.
void publish_quat() {

  tf2::Quaternion q;
		
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }

  odom_data_pub_quat.publish(quatOdom);
}

// Update odometry information
//TODO: update with correct math
void update_odom() {
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
  
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);

  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
	
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}

  // Calculate the new pose (x, y, and theta)
  //TODO: calculate pose with omni-wheel math
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
	else{}

  // Compute the velocity
  //TODO: compute velocity with correct math
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}

int main(int argc, char** argv){
    // Set the data fields of the odometry message
    odomNew.header.frame_id = "odom";
    odomNew.pose.pose.position.z = 0;
    odomNew.pose.pose.orientation.x = 0;
    odomNew.pose.pose.orientation.y = 0;
    odomNew.twist.twist.linear.x = 0;
    odomNew.twist.twist.linear.y = 0;
    odomNew.twist.twist.linear.z = 0;
    odomNew.twist.twist.angular.x = 0;
    odomNew.twist.twist.angular.y = 0;
    odomNew.twist.twist.angular.z = 0;
    odomOld.pose.pose.position.x = initialX;
    odomOld.pose.pose.position.y = initialY;
    odomOld.pose.pose.orientation.z = initialTheta;
    
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle n;

    ros::Subscriber subForRightCounts = n.subscribe("right_RPMs", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = n.subscribe("left_RPMs", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subInitialPose = n.subscribe("initial_2d", 1, set_initial_2d);

    // Publisher of simple odom message where orientation.z is an euler angle
    odom_data_pub = n.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
    // Publisher of full odom message where orientation is quaternion
    odom_data_pub_quat = n.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
    
    ros::Rate loop_rate(30); 
	
    while(ros::ok()) {
    
        if(initialPoseRecieved) {
            update_odom();
            publish_quat();
        }
        ros::spinOnce();
        loop_rate.sleep();
  }
  return 0;
}