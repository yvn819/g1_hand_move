# g1_arm_joint_control

ROS 2 package to command Unitree G1 arm joints by target joint angles and run a local MuJoCo sim node.

## Topics

- Subscribed command topic: `/g1/arm_joint_position_cmd` (`std_msgs/Float64MultiArray`)
- Published low-level topic: `/lowcmd` (`unitree_hg/LowCmd`)
- Sim state topic: `/lowstate` (`unitree_hg/LowState`)

The simulator node is minimal: no waypoint loading/following.
Base can be pinned in place with `lock_base: true` (default in `arm_sim.yaml`).

## Arm Command Order (14 DOF)

`[L_SHOULDER_PITCH, L_SHOULDER_ROLL, L_SHOULDER_YAW, L_ELBOW, L_WRIST_ROLL, L_WRIST_PITCH, L_WRIST_YAW, R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_SHOULDER_YAW, R_ELBOW, R_WRIST_ROLL, R_WRIST_PITCH, R_WRIST_YAW]`

Mapped to LowCmd motor indices `[15..28]`.

## Build

```bash
cd /home/yutong/scaledrive.ai/fincantieri/g1_hand_move
colcon build --packages-select g1_arm_joint_control
```

## Environment Setup (every terminal)

```bash
source /opt/ros/humble/setup.bash
source /home/yutong/scaledrive.ai/fincantieri/g1_hand_move/install/setup.bash
```

`unitree_hg` is expected to be built in this workspace (`g1_hand_move/src/unitree_hg`).

## Run Arm Controller Only

```bash
ros2 launch g1_arm_joint_control arm_control.launch.py
```

## Run MuJoCo Sim + Arm Controller (single launch)

```bash
ros2 launch g1_arm_joint_control arm_mujoco_sim.launch.py
```

This launch starts:
- `g1_arm_joint_control/arm_sim_node`
- `g1_arm_joint_control/arm_joint_controller`

If your MuJoCo dependencies are not in conda, disable the prefix:

```bash
ros2 launch g1_arm_joint_control arm_mujoco_sim.launch.py use_conda_prefix:=false
```

## Send a Test Arm Pose

```bash
ros2 run g1_arm_joint_control arm_pose_sender --ros-args \
  -p joint_angles:="[0.2,0.3,0.0,0.7,0.0,0.0,0.0,0.2,-0.3,0.0,0.7,0.0,0.0,0.0]" \
  -p repeat_count:=100
```

## Continuous Arm Motion

Continuous shoulder-yaw rotation (left/right mirrored):

```bash
ros2 run g1_arm_joint_control arm_pose_sender --ros-args \
  -p motion_mode:=yaw_rotate \
  -p yaw_amplitude:=0.6 \
  -p yaw_frequency_hz:=0.2
```
