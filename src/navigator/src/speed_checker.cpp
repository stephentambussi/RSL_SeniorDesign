#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

//TODO: Calculate velocity from acceleration
//TODO: Figure out how to filter values via covariance matrix (if it is not being done already)
//TODO: put a limit on the robots speed (max speed is 5mph)

/**
 * Subscriber callbacks
 */

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_sensors_subscriber");
    ros::NodeHandle n;

    ros::Subscriber subImu = n.subscribe("/zed2i/zed_node/imu/data", 10, imuCallback);

    ros::spin();

    return 0;
}