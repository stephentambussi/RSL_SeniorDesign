#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <unistd.h>
#include <chrono>

float max_speed = 2.23; //in meters/s (~= 5mph)
float max_var = 0.5; //experimental - greatest amount of variance allowed for variables
auto start = std::chrono::steady_clock::now(); //timer check
auto finish = std::chrono::steady_clock::now();
//TODO: Calculate velocity from acceleration

/**
 * Subscriber callbacks
 */

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    /*
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    */

    /* Linear Acceleration and Variance */
    //ROS_INFO("Linear Accel: x = %.3f[m/s^2], y = %.3f[m/s^2]", msg->linear_acceleration.x, msg->linear_acceleration.y);
    //Row major x
    //ROS_INFO("Linear Accel Var: x[%.3f] [m/s^2]", msg->linear_acceleration_covariance[0]);
    //Row major y
    //ROS_INFO("Linear Accel Var: y[%.3f] [m/s^2]", msg->linear_acceleration_covariance[4]);

    /* Angular Acceleration and Variance */
    //ROS_INFO("Ang. vel. (z): %.3f [deg/sec]", msg->angular_velocity.z);
    //Row major about z axis
    //ROS_INFO("Ang. vel. Var: z[%.3f] [deg/sec]", msg->angular_velocity_covariance[8]);
    //usleep(100000); //to slow down output
    
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