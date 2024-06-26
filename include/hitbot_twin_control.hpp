#ifndef HITBOT_TWIN_CONTROL_H
#define HITBOT_TWIN_CONTROL_H

#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <modbus.h>
#include <chrono>
#include <unistd.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>

#include <string>
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
    ros::NodeHandle *_nh;
    void encoder_read();
    double degrees_radians_normalized(double);
    double linear_map(double, double, double, double, double);
    void publish_joint_command();
    void publish_claw_command();
    std::string node_name;

    ros::Publisher joints_command_pub, joints_state_cmd_pub;
    ros::Publisher claw_command_pub;
    sensor_msgs::JointState joint_command;

    // control command for hitbot
    double twin_angles[4];
    double claw_open_dist;
    std::vector<uint16_t> read_registers(modbus_t *, int, int, int);
    void write_float_to_slave(modbus_t *, int, int, float);
    double round_number_to_two_decimal(double);
};

#endif // HITBOT_TWIN_CONTROL_H