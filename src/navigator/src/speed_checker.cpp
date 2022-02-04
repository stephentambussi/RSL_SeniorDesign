#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>

ros::Publisher s; //publisher for linang correction array
std_msgs::Float32MultiArray correct_linang;

float max_speed = 2.23; //in meters/s (~= 5mph)
float max_var = 0.5; //experimental - greatest amount of variance allowed for variables

class SpeedCheck
{
    /* Private member variables */
    float lin_vel_x;
    float lin_vel_y;
    float ang_vel_z;
    float time_elapsed; //in seconds
    float correct_linx; //correction value if cobot speed is too fast
    float correct_liny;
    float correct_angz;

    public:
        SpeedCheck() //default constructor
        {
            lin_vel_x = 0; //initial velocity
            lin_vel_y = 0; //initial velocity
            ang_vel_z = 0;
            time_elapsed = 0.25; //calculating velocity every quarter second
            /* Initialize the below to -1 so that the subscriber node knows
            *  not to change current velocity because it is below max_speed */
            correct_linx = -1; 
            correct_liny = -1;
            correct_angz = -1;
        }

        /*
        * Subscriber callbacks
        */
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
        {
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

            lin_vel_x = lin_vel_x + (msg->linear_acceleration.x * time_elapsed); //v = u + at
            lin_vel_y = lin_vel_y + (msg->linear_acceleration.y * time_elapsed);
            ang_vel_z = msg->angular_velocity.z;

            ROS_INFO("Linear Velocity: x = %.3f [m/s], y = %.3f [m/s]", lin_vel_x, lin_vel_y);
            
            /* If any of the velocities are past the max_speed, then cap the speed to the max */
            /* Conditional operator is used to maintain direction of cobot */
            if(abs(lin_vel_x) > max_speed) 
                correct_linx = ((lin_vel_x < 0) ? (lin_vel_x + (abs(lin_vel_x) - max_speed)) : (lin_vel_x - (abs(lin_vel_x) - max_speed)));

            if(abs(lin_vel_y) > max_speed)
                correct_liny = ((lin_vel_y < 0) ? (lin_vel_y + (abs(lin_vel_y) - max_speed)) : (lin_vel_y - (abs(lin_vel_y) - max_speed)));

            if(abs(ang_vel_z) > max_speed)
                correct_angz = ((ang_vel_z < 0) ? (ang_vel_z + (abs(ang_vel_z) - max_speed)) : (ang_vel_z - (abs(ang_vel_z) - max_speed)));

            /* If any of the above conditions are true, then publish the speed cap */
            if(abs(lin_vel_x) > max_speed || abs(lin_vel_y) > max_speed || abs(ang_vel_z) > max_speed)
            {
                correct_linang.data.push_back(correct_linx);
                correct_linang.data.push_back(correct_liny);
                correct_linang.data.push_back(correct_angz);

                p.publish(correct_linang);
            }
        }
};

/**
 * Node main function
 */
int main(int argc, char** argv) {
    SpeedCheck checker(); //initialize class object with default constructor
    ros::init(argc, argv, "zed_sensors_subscriber");
    ros::NodeHandle n;

    ros::Subscriber subImu = n.subscribe("/zed2i/zed_node/imu/data", 10, &SpeedCheck::imuCallback, &checker);
    s = n.advertise<std_msgs::Float32MultiArray>("linang_correction", 10);

    while(true)
    {
        //TODO: look into ros rate.sleep instead of usleep
        usleep(250000); //stop execution for 0.25 seconds
        ros::spinOnce(); //call imuCallback method every 0.25s (4 times a second)
    }
    //ros::spin();
    return 0;
}