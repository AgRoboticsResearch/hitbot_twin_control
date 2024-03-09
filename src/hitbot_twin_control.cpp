#include "hitbot_twin_control.hpp"
#include <iostream>

HITBOT_TWIN_CONTROL::HITBOT_TWIN_CONTROL(ros::NodeHandle &nh)
{
    /* Setting parameters */
    _nh = &nh;
    // the topic name depend on the controller's name
    joints_command_pub = _nh->advertise<sensor_msgs::JointState>("/hitbot_joint_group_position_controller/command", 10);

    this->init();
}

void HITBOT_TWIN_CONTROL::init()
{
    ROS_INFO("Hitbot Twin Control Initializing...");
    node_name = ros::this_node::getName();
    std::string init_msg = "";
    // initialize the reading of the encoder
    // the port is renamed based upon the env id
    ctx = modbus_new_rtu("/dev/ttyUSB_fake_robot", 115200, 'N', 8, 1);
    if (ctx == nullptr)
    {
        init_msg = "Unable to create the libmodbus context\n";
        ROS_INFO("%s %s", node_name.c_str(), init_msg.c_str());
    }
    if (modbus_connect(ctx) == -1)
    {
        init_msg = "Connection failed!! ";
        ROS_INFO("%s %s %s", node_name.c_str(), init_msg.c_str(), modbus_strerror(errno));
        // modbus_strerror(errno) << "\n";
        // release the ctx
        modbus_free(ctx);
    }
    // why??
    uint32_t to_sec = 1;    // s
    uint32_t to_usec = 100; // us
    modbus_set_response_timeout(ctx, to_sec, to_usec);
}

void HITBOT_TWIN_CONTROL::publish_joint_command()
{

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now(); // Current time

    // TODO: Replace the next lines with actual encoder reading and conversion logic
    // Dummy values for demonstration
    joint_state.position.assign(twin_angles, twin_angles + 4);
    // joint_state.position = twin_angles; // Example value for joint1

    // Publish the joint state message
    joints_command_pub.publish(joint_state);
}

double HITBOT_TWIN_CONTROL::degrees_radians_normalized(double degrees)
{
    double radians = degrees * (M_PI / 180.0);

    // Normalize the angle to the range -pi to pi using fmod
    radians = std::fmod(radians, 2 * M_PI);

    // Adjust values to be within -pi to pi
    if (radians > M_PI)
    {
        radians -= 2 * M_PI;
    }
    else if (radians < -M_PI)
    {
        radians += 2 * M_PI;
    }

    return radians;
}

double HITBOT_TWIN_CONTROL::linear_map(double value, double src_range_start, double src_range_end, double dest_range_start, double dest_range_end)
{
    double ret = 0;

    if (src_range_start == src_range_end)
    {
        std::cerr << "Source range cannot be zero" << std::endl;
    }
    ret = dest_range_start + (dest_range_end - dest_range_start) * ((value - src_range_start) / (src_range_end - src_range_start));
    return ret;
}

void HITBOT_TWIN_CONTROL::encoder_read()
{
    std::vector<uint16_t> read_value_from_register = read_registers(ctx, 2, 0x00, 5);
    if (read_value_from_register[0] != 0 && read_value_from_register[1] != 0)
    {

        twin_angles[2] = degrees_radians_normalized(linear_map(read_value_from_register[2], 3994, 1195, 90, -90));
        twin_angles[1] = degrees_radians_normalized(linear_map(read_value_from_register[1], 1480, 960, 0, 30));
        // round to two decimals
        twin_angles[1] = degrees_radians_normalized(std::round(twin_angles[1] * 100.0) / 100.0);
        twin_angles[4] = degrees_radians_normalized(linear_map(read_value_from_register[4], 3400, 720, -90, 90));
        twin_angles[3] = degrees_radians_normalized(linear_map(read_value_from_register[3], 900, 3560, 90, -90));
        twin_angles[0] = degrees_radians_normalized(linear_map(read_value_from_register[0], 650, 3000, -10, -160));
        // twin_angles[2] = twin_angles[2] + twin_angles[3] + twin_angles[4] - 28;
    }
}

// read registers
std::vector<uint16_t> HITBOT_TWIN_CONTROL::read_registers(modbus_t *ctx, int slave_id, int addr, int nb)
{
    modbus_set_slave(ctx, slave_id); // set slave addr

    std::vector<uint16_t> tab_reg(nb, 0);
    int rc = modbus_read_input_registers(ctx, addr, nb, &tab_reg[0]);
    if (rc == -1)
    {
        // std::cerr << "Read failed from slave " << slave_id << ": " << modbus_strerror(errno) << "\n";
        ROS_INFO("%s: Read failed from slave %d: %s", node_name.c_str(), slave_id, modbus_strerror(errno));

        return std::vector<uint16_t>(); // empty return means failure
    }

    // return read value
    return tab_reg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hitbot_twin_controller");
    ros::NodeHandle nh("~");

    ros::Rate rate(300.0);
    HITBOT_TWIN_CONTROL twin_control(nh);
    while (ros::ok())
    {

        // read encoder
        twin_control.encoder_read();
        // publish it as joint_state msg
        twin_control.publish_joint_command();
        rate.sleep();
    }

    return 0;
}