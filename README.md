# hitbot_twin_control
control the hitbot with twin robot arm
## Commands:
 You can test some function through simple commands below.

```bash
# start real robot
roslaunch hitbot_bringup real_arm.launch 

```

```bash
# run hitbot_twin_robot
rosrun hitbot_twin_control hitbot_twin_control_node
```

```bash
# change controller
rosservice call /controller_manager/switch_controller "start_controllers: ['hitbot_joint_group_position_controller']
stop_controllers: ['hitbot_joint_traj_controller']
strictness: 0
start_asap: false
timeout: 0.0"
```
