#include "hitbot_twin_encoder.hpp"

HITBOT_TWIN_CONTROL::HITBOT_TWIN_CONTROL(ros::NodeHandle &nh)
{
    /* Setting parameters */
    _nh = &nh;
    // the topic name depend on the controller's name
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/hitbot_joint_group_position_controller/command", 10);

    this->init();
}

void HITBOT_TWIN_CONTROL::init()
{
    ROS_INFO("Hitbot Twin Control Initializing...");

    // initialize the reading of the encoder
    ctx = modbus_new_rtu("/dev/ttyUSB_fake_robot", 115200, 'N', 8, 1);
    if (ctx == nullptr)
    {
        ROS_INFO("Unable to create the libmodbus context\n");
    }
    if (modbus_connect(ctx) == -1)
    {
        ROS_INFO("Connection failed!!\n")
        // modbus_strerror(errno) << "\n";
        // release the ctx
        modbus_free(ctx);
    }
    uint32_t to_sec = 1;    // s
    uint32_t to_usec = 100; // us
    modbus_set_response_timeout(ctx, to_sec, to_usec);
}

double HITBOT_TWIN_CONTROL::degreesToRadiansNormalized(double degrees)
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

double HITBOT_TWIN_CONTROL::linearMap(double value, double srcRangeStart, double srcRangeEnd, double destRangeStart, double destRangeEnd)
{
    double ret = 0;

    if (srcRangeStart == srcRangeEnd)
    {
        std::cerr << "Source range cannot be zero" << std::endl;
        return 0;
    }
    ret = destRangeStart + (destRangeEnd - destRangeStart) * ((value - srcRangeStart) / (srcRangeEnd - srcRangeStart));
    // 线性映射公式
    return ret;
}

void HITBOT_TWIN_CONTROL::encoder_read(const ros::Time &time, const ros::Duration &period)
{
    float converted_angle;
    // update the encoder reading
    rbt->get_robot_real_coor();

    // get current pos
    pos[0] = rbt->real_z / 1000.0;
    pos[1] = rbt->real_angle1 * DEG_TO_RAD;
    pos[2] = rbt->real_angle2 * DEG_TO_RAD;
    converted_angle = rbt->real_rotation - rbt->real_angle1 - rbt->real_angle2 + EE_OFFSET;
    pos[3] = converted_angle * DEG_TO_RAD;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hitbot_ros_controller");
    ros::NodeHandle nh("~");

    HITBOT_TWIN_CONTROL robot(nh);

    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(300.0);

    while (ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        // auto start_time = std::chrono::high_resolution_clock::now();
        robot.read(time, period);
        cm.update(time, period);
        robot.write(time, period);
        // auto end_time = std::chrono::high_resolution_clock::now();

        // // 计算运行时间
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // std::cout << "程序运行时间: " << duration.count() << " 微秒" << std::endl;
        rate.sleep();
    }

    robot.dinit();

    return 0;
}