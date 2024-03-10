#include "hitbot_twin_control.hpp"
#include <iostream>

HITBOT_TWIN_CONTROL::HITBOT_TWIN_CONTROL(ros::NodeHandle &nh)
{
    /* Setting parameters */
    _nh = &nh;
    // the topic name depend on the controller's name
    // joints_command_pub = _nh->advertise<sensor_msgs::JointState>("/hitbot_joint_traj_controller/command", 10);
    // joints_command_pub = _nh->advertise<trajectory_msgs::JointTrajectory>("/hitbot_joint_traj_controller/command", 10);
    joints_command_pub = _nh->advertise<std_msgs::Float64MultiArray>("/hitbot_joint_group_position_controller/command", 1);
    claw_command_pub = nh.advertise<std_msgs::Float64>("/hitbot_claw_position", 1);

    this->init();
}

void HITBOT_TWIN_CONTROL::init()
{
    ROS_INFO("Hitbot Twin Control Initializing...");
    node_name = ros::this_node::getName();
    std::string init_msg = "";
    // initialize the reading of the encoder
    // the port is renamed based upon the env id
    ctx = modbus_new_rtu("/dev/ttyUSB_twin_robot", 115200, 'N', 8, 1);
    if (ctx == nullptr)
    {
        init_msg = "Unable to create the libmodbus context\n";
        ROS_INFO("%s %s", node_name.c_str(), init_msg.c_str());
        // TOOD: add ros error here

    }
    if (modbus_connect(ctx) == -1)
    {
        init_msg = "Connection failed!! ";
        ROS_INFO("%s %s %s", node_name.c_str(), init_msg.c_str(), modbus_strerror(errno));
        // modbus_strerror(errno) << "\n";
        // release the ctx
        modbus_free(ctx);
        // TOOD: add ros error here

    }

    // why??
    uint32_t to_sec = 1;    // s
    uint32_t to_usec = 100; // us
    modbus_set_response_timeout(ctx, to_sec, to_usec);

    ROS_INFO("Hitbot Twin Control Initialized!!");
}
void HITBOT_TWIN_CONTROL::publish_claw_command()
{

    std_msgs::Float64 claw_dist;
    claw_dist.data = claw_open_dist;
    claw_command_pub.publish(claw_dist);
}
void HITBOT_TWIN_CONTROL::publish_joint_command()
{

    std_msgs::Float64MultiArray joint_cmd;
    // sensor_msgs::JointState joint_cmd;
    // trajectory_msgs::JointTrajectory joint_cmd;
    // trajectory_msgs::JointTrajectoryPoint point;
    // joint_cmd.header.stamp = ros::Time::now(); // Current time
    // joint_cmd.joint_names = {
    //     "joint1",
    //     "joint2",
    //     "joint3",
    //     "joint4",
    // };
    // TODO: Replace the next lines with actual encoder reading and conversion logic

    // Dummy values for demonstration
    // joint_cmd.position.assign(twin_angles, twin_angles + 4);
    joint_cmd.data.insert(joint_cmd.data.end(), std::begin(twin_angles), std::end(twin_angles));
    // point.positions.assign(twin_angles, twin_angles + 4);
    // joint_cmd.points.push_back(point);
    // Publish the joint state message
    joints_command_pub.publish(joint_cmd);
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

        twin_angles[1] = degrees_radians_normalized(linear_map(read_value_from_register[3], 800, 3460, 90, -90));
        twin_angles[3] = degrees_radians_normalized(linear_map(read_value_from_register[2], 3994, 1195, 90, -90));
        twin_angles[2] = degrees_radians_normalized(linear_map(read_value_from_register[4], 3400, 560, -90, 90));
        twin_angles[0] = linear_map(read_value_from_register[0], 650, 3000, -10, -160)/1000;
        
        // E-claw
        claw_open_dist = degrees_radians_normalized(linear_map(read_value_from_register[1], 1480, 800, 0, 30));
        claw_open_dist = degrees_radians_normalized(std::round(claw_open_dist * 100.0) / 100.0);
        // twin_angles[2] = twin_angles[2] + twin_angles[3] + twin_angles[4] - 28;
    }
    // std::cout << "Register values from angle : ";
    // for (float reg : twin_angles)
    // {
    //     std::cout << reg << " ";
    // }
    // std::cout << "\n";
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
    ros::Rate rate(500.0);
    
    HITBOT_TWIN_CONTROL twin_control(nh);
    while (ros::ok())
    {

        // read encoder
        twin_control.encoder_read();
        // publish it as joint_state msg
        twin_control.publish_joint_command();
        twin_control.publish_claw_command();
        rate.sleep();
    }

    return 0;
}