#ifndef HITBOT_ROS_CONTROL
#define HITBOT_ROS_CONTROL

#include "hitbot_interface.h"
#include <iostream>
#include <cmath>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <modbus.h>

#include <sensor_msgs/JointState.h>
/**
 * @brief This class wraps the functions of unitree-arm-sdk
 * for controlling HITBOT robots into the ros_control framework.
 */
class HITBOT_TWIN_CONTROL
{
public:
    HITBOT_TWIN_CONTROL(ros::NodeHandle &nh);

    void init();
    modbus_t *ctx;
    void encoder_read();
    double degreesToRadiansNormalized(double degrees);
    std::string node_name = ros::this_node::getName();

    ros::Publisher joints_command_pub;

private:
    // control command for hitbot
    double joint_cmd[4];

    ros::NodeHandle *_nh;
    float goal_angle1;
    float goal_angle2;
    float goal_z;
    float goal_r;
    float goal_speed;
    float rough;
    double DEG_TO_RAD = M_PI / 180.0;
    double EE_OFFSET = 28.0; // deg
};

#endif // Hitbot_ROS_control