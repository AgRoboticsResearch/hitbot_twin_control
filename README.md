# hitbot_twin_control
control the hitbot with twin robot arm
## Commands:
 You can test some function through simple commands below.

## Dependency
```bash
sudo apt-get install libmodbus-dev
# REMOVE brltty if you are in ubuntu 22.04 and can not find CH340(USB to 232) in /dev/ttyUSB
sudo apt remove brltty
```
### start real robot hardware
```bash
# start real robot
roslaunch hitbot_bringup real_arm.launch 

```
### start twin controller
```bash
# run twin controller
rosrun hitbot_twin_control hitbot_twin_control_node
```
### switch controller type
```bash
# switch controller from trajectory controller to position controller
rosservice call /controller_manager/switch_controller "start_controllers: ['hitbot_joint_group_position_controller']
stop_controllers: ['hitbot_joint_traj_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```
### twin controller commands
```bash
- arm-joints command:/hitbot_joint_group_position_controller/command(Float64MultiArray)
- gripper command: /hitbot_claw_position(Float64)
```

### usb device rule files
- create two files in '/etc/udev/rules.d/' with the name as:
    - ee_gripper.rules
    ```bash
    SUBSYSTEM=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="ttyUSB_girpper"
    ```
    - fake_robot.rules
    ```bash
     SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="ttyUSB_twin_robot"
     ```

