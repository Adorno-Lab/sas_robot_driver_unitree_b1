![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_b1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)


# sas_robot_driver_unitree_b1

SmartArmStack driver of the **Unitree B1** in high-level control: Unitree's locomotion controller moves the legs
with the target twist. The communication with the robot uses UDP, through
[unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) (branch `B1`, a submodule of this
repository).

The driver follows the legged robot driver standard of [sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation) (branch `new_generation`): it
implements `LeggedRobotDriver` and runs with `LeggedRobotDriverROS`. A control package that uses
`LeggedRobotDriverClient` works with this robot and with any other legged robot that follows the standard.

## Limbs

The B1 is a floating base with four legs. Each leg is a standard SAS robot driver on `sas_b1/b1_1/<limb>`.

| Limb | Joints | Accepts targets |
|---|---|---|
| `front_right_leg` | FR_hip_joint, FR_thigh_joint, FR_calf_joint | never (moved by the locomotion controller) |
| `front_left_leg` | FL_hip_joint, FL_thigh_joint, FL_calf_joint | never (moved by the locomotion controller) |
| `rear_right_leg` | RR_hip_joint, RR_thigh_joint, RR_calf_joint | never (moved by the locomotion controller) |
| `rear_left_leg` | RL_hip_joint, RL_thigh_joint, RL_calf_joint | never (moved by the locomotion controller) |

- The joint names are the ones of `b1_description/xacro/b1.urdf` of
  [unitree_ros](https://github.com/unitreerobotics/unitree_ros), in the order of unitree_legged_sdk (hip, thigh, calf).
- The joint limits are the ones of `b1_const.h` of unitree_legged_sdk (identical in `b1.urdf`): hip [-0.75, 0.75],
  thigh [-1.0, 3.5], calf [-2.6, -0.6] rad.

## Modes and capabilities

| Mode | B1 behaviour |
|---|---|
| `IDLE` (initial mode) | Zero velocity. |
| `STANDING` | The robot stops and stands (FORCE_STAND). The base orientation and the base height are accepted. |
| `WALKING` | The twist is accepted. |

- Every mode change first stops the robot (zero velocity), and resets the base orientation and height.
- Entering `STANDING`: the robot walks with zero velocity until its velocity is below 0.1, and then stands. From then
  on, it stays in FORCE_STAND whatever its measured velocity.
- Leaving `STANDING` (to another mode, or when the driver stops): the robot first stands in the neutral pose for 1 s,
  as in `example_walk.cpp` of unitree_legged_sdk, so that it never walks with a tilted body.
- The twist: `vx`, `vy`, and `wz` (yaw rate), in the body frame. They are saturated to the ranges of the robot:
  `vx` [-0.8, 1.2] m/s, `vy` [-0.25, 0.25] m/s, `wz` [-0.75, 0.75] rad/s.
- The base orientation: ZYX roll, pitch, and yaw relative to the pose at the start of `STANDING` (the frame F_f of
  sas_tools), taken by the B1 as the angles of its standing pose. The limits (published in `get/info`) are roll
  [-0.3, 0.3], pitch [-0.3, 0.3], and yaw [-0.6, 0.6] rad.
- The base height, in meters above the ground: the B1 takes its difference with the nominal height, 0.58 m, saturated
  to [-0.16, 0.16] m. Thus, the height is within [0.42, 0.74] m. It is only accepted in `STANDING`
  (`supports_base_height_while_walking` is false in `get/info`).
- With zero velocity, the robot walks in place. With `FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO`, it
  stands instead, once its velocity is below 0.1 (see
  [#4](https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/issues/4)).
- With `mode: "None"`, the driver only reads the state of the robot: only `IDLE` is supported, and nothing is
  commanded.
- `get/twist` is the twist of the base estimated by the B1 (forward and side velocities, and yaw rate, in the body
  frame), in every mode. Pose estimators (e.g. an EKF with Vicon) use it as the motion of the base.

## Topics

| Prefix | Topics |
|---|---|
| `sas_b1/b1_1/` (base) | `get/info`, `get/status`, `get/imu`, `get/twist` (measured), `set/high_level_mode`, `set/target_twist`, `set/watchdog_trigger`, `set/shutdown` |
| `sas_b1/b1_1/<limb>/` | the standard SAS joint topics: `get/joint_states`, `get/joint_positions_min`, `get/joint_positions_max`, `set/target_joint_positions`, `set/shutdown` |

See the [topic standard of sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation#topic-standard) for the message types,
the client roles, and the watchdog.

> [!IMPORTANT]
> The topics of the previous version of this driver (`get/FR_joint_states`, `get/IMU_state`, `get/twist_state`,
> `get/pose_state`, `get/battery_state`, `get/imu_orientation`, `set/holonomic_target_velocities`) are no longer
> published. Use `LeggedRobotDriverClient` of sas_tools instead. The odometry, the measured twist, and the battery
> state are not published yet.

## Install

Requires ROS 2 Jazzy, [DQ Robotics](https://github.com/dqrobotics/cpp), the SmartArmStack packages (`sas_core`,
`sas_common`, `sas_robot_driver`), and:

- [sas_tools](https://github.com/Adorno-Lab/sas_tools), in the same workspace.
- `libmarinholab-sas-core`, installed in the system. It provides the threads of the driver (`ThreadManager`).

```shell
cd ~/ros2_ws/src
git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1 --recursive
cd ~/ros2_ws
colcon build --packages-up-to sas_robot_driver_unitree_b1
```

The shared library of unitree_legged_sdk (amd64 or arm64) is installed with the driver, so `LD_LIBRARY_PATH` does
not need to be changed.

## Usage

Run the driver in a different terminal window or tab. Be ready to close it, as it activates the real robot if the
connection is successful.

> [!CAUTION]
> The robot could move with the next command. Be ready to perform an emergency stop!

```shell
ros2 launch sas_robot_driver_unitree_b1 real_b1_white_robot_launch.py
```

To stop the robot, press `Ctrl + C`. The driver stops the robot (a quick stop, or lying it down with
`LIE_DOWN_ROBOT_WHEN_DEINITIALIZE`) before closing the communication.

| Launch file | Robot | Node |
|---|---|---|
| `real_b1_white_robot_launch.py` | white B1 (192.168.8.170) | `sas_b1/b1_1` |
| `real_b1_black_robot_launch.py` | black B1 (192.168.8.226) | `sas_b1/b1_2` |
| `real_b1_white_robot_echo_launch.py`, `real_b1_black_robot_echo_launch.py` | the same robots with `mode: "None"` (state only) | the same nodes |
| `real_b1z1_white_robot_launch.py`, `real_b1z1_black_robot_launch.py` | the B1 and its Z1 arm (requires `sas_robot_driver_unitree_z1`) | the same nodes |

The node name and namespace in the launch file (e.g. `b1_1` in `sas_b1`) define the topic prefix, `sas_b1/b1_1`.
The launch files remap the `set/shutdown` of the base to `/sas/set/shutdown`, the topic of the emergency stop of the
lab, as the launch files of the Z1 driver: one message on it stops every driver on the same `ROS_DOMAIN_ID`. The
`set/shutdown` of each leg is not remapped.

| Node parameter | Default | Description |
|---|---|---|
| `ROBOT_IP` | mandatory | Address of the robot, e.g. `192.168.123.220` (LAN) or its address in the WiFi network |
| `ROBOT_PORT` | mandatory | Port of the high-level controller of the robot, `8082` |
| `LIE_DOWN_ROBOT_WHEN_DEINITIALIZE` | mandatory | `true`: the robot lies down when the driver stops (about 12 s). `false`: a quick stop (1.5 s) |
| `thread_sampling_time_sec` | mandatory | Period of the control loop of `LeggedRobotDriverROS`, e.g. `0.002` |
| `mode` | `"VelocityControl"` | `"VelocityControl"` (the robot walks with the target twist) or `"None"` (state only) |
| `LOCAL_PORT` | `8090` | Port of this computer |
| `FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO` | `false` | Stand instead of walking in place with zero velocity |
| `twist_timeout_sec` | `0.2` | A zero twist is sent when no twist arrives within this time |

To command the robot, use the clients of sas_tools with these launch parameters (see the
[example in sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation#commanding-the-robot)):

```yaml
legged_robot_prefix: "sas_b1/b1_1"
limb_prefixes: ["sas_b1/b1_1/front_right_leg", "sas_b1/b1_1/front_left_leg",
                "sas_b1/b1_1/rear_right_leg", "sas_b1/b1_1/rear_left_leg"]
```

## Testing with ROS

Use `sas_legged_client_window` of [sas_tools](https://github.com/Adorno-Lab/sas_tools): a Qt window that monitors or
commands the running driver through the clients of sas_tools, as a controller would. It shows the status, the mode, the
IMU, and the joint states of the legs, and it has the mode buttons, sliders for the twist and the base pose, an
emergency stop, and a watchdog.

```shell
ros2 launch sas_legged_client_window sas_legged_client_window_launch.py legged_robot_prefix:=sas_b1/b1_1
```

For the Z1 arm too, add `own_driver_limb_prefixes:="['sas_z1/z1_1']"`. See its README for the parameters (control or
monitor mode, watchdog).

## Testing without ROS

`qt/configuration_window` is a Qt application that runs `DriverUnitreeB1` (the driver without ROS) with sliders for
the twist and for the standing pose, and displays the state of the robot. The dial selects walking (0) or standing (1).
Turning it resets the sliders, as the driver resets its targets. Open `qt/configuration_window/CMakeLists.txt` in Qt Creator, or:

```shell
cmake -S qt/configuration_window -B qt/configuration_window/build -DCMAKE_PREFIX_PATH=~/Qt/6.8.1/gcc_64
cmake --build qt/configuration_window/build
```

## Docker

> [!WARNING]
> The Docker image has not been updated for this version of the driver yet (it does not install sas_tools or
> `libmarinholab-sas-core`).

#### Prerequisites:
- Docker with sudo permissions.
- Prepare the Unitree B1 robot.
- Connect your machine to the WiFi B1 network (unitree-b1-5g-xx)

1. Clone this repository
```shell
cd ~/Downloads
git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1 --recursive
cd sas_robot_driver_unitree_b1
```
2. Build the docker image

> [!IMPORTANT]
> The argument of the following command sets the ROS_DOMAIN_ID

```shell
sh build_sas_rd_unitree_b1_docker.sh 1
```
3. Start the docker container
```shell
sh start_sas_rd_unitree_b1_docker.sh
```
4. Build the ROS2 packages
```shell
buildros2
```
5. Start the driver
> [!CAUTION]
> The robot could move with the next command. Be ready to perform an emergency stop!

```shell
launch_ROS2_drivers
```

![start_ROS2_drivers](https://github.com/user-attachments/assets/c30a7867-d738-4423-970a-9a18e476b080)

6. To stop the robot, press `Ctrl + C`

![ctrlc](https://github.com/user-attachments/assets/a206fc3e-5065-406d-9a82-2624c197b9a3)

![b1_stop](https://github.com/user-attachments/assets/e9d26807-dccf-4afe-b802-451619140593)
