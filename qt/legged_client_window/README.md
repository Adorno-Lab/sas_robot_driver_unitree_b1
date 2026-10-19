# legged_client_window

Qt application to test a running legged robot driver of [sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation)
(e.g. the Unitree B1, optionally with the Z1 arm) through ROS 2, as a controller would: it uses
`LeggedRobotDriverClient` for the base and one `RobotDriverClient` per limb.

It shows the status, the high-level mode, the IMU, and the joint states of every limb, and it has the mode buttons,
sliders for the twist, and an emergency stop. The limbs are built from `get/info` of the driver, so the window does
not depend on the robot.

> [!NOTE]
> This is not a colcon package (colcon does not look for packages inside the driver package). It is a ROS 2 node
> built with plain CMake: build it and run it in a terminal in which ROS 2 and the workspace are sourced.

## Build

Requires Qt 6, and the workspace that contains `sas_tools`, already built with colcon.

```shell
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/sas_robot_driver_unitree_b1/qt/legged_client_window
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/Qt/6.8.1/gcc_64;$CMAKE_PREFIX_PATH"
cmake --build build
```

With Qt Creator, start it from that sourced terminal (e.g. `~/Qt/Tools/QtCreator/bin/qtcreator &`) and open
`CMakeLists.txt`. Otherwise, CMake does not find `rclcpp` or `sas_tools`.

## Run

1. Start the driver (in another terminal), e.g.:

   ```shell
   ros2 launch sas_robot_driver_unitree_b1 real_b1_white_robot_launch.py
   ```

2. In a sourced terminal (the ROS 2 libraries are only found there), with the same `ROS_DOMAIN_ID` as the driver:

   ```shell
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   cd ~/ros2_ws/src/sas_robot_driver_unitree_b1/qt/legged_client_window
   ./build/legged_client_window
   ```

Other robots and options:

```shell
# The black B1
./build/legged_client_window --ros-args -p legged_robot_prefix:=sas_b1/b1_2

# The B1 with the Z1 arm (sas_robot_driver_unitree_z1 must be running too)
./build/legged_client_window --ros-args -p own_driver_limb_prefixes:="['sas_z1/z1_1']"

# Only read the robot, e.g. next to a controller (the emergency stop still works)
./build/legged_client_window --ros-args -p monitor_only:=true
```

| Parameter | Default | Description |
|---|---|---|
| `legged_robot_prefix` | `sas_b1/b1_1` | Topic prefix of the legged driver (namespace and node name of its launch file) |
| `own_driver_limb_prefixes` | `[]` | Limbs with their own driver, e.g. `['sas_z1/z1_1']`. Their widgets appear once the legged driver and their joint states are received |
| `monitor_only` | `false` | Only read the robot: the command topics are not created |

## Usage

> [!CAUTION]
> The robot moves with the sliders of the twist. Be ready to perform an emergency stop!

1. Wait until the status shows *Connected*. The modes that the robot does not support are disabled (the B1 only
   supports `IDLE` and `WALKING`).
2. Select `WALKING`. The twist sliders are enabled once the robot reports `WALKING`.
3. Move the sliders. The twist is sent every 50 ms; if the window stops sending it (e.g. it freezes or is closed),
   the driver stops the robot after its twist timeout (0.2 s by default).
4. Press `ZERO` to stop, and select `IDLE` when done.

- The sliders go back to zero on every mode request, and whenever the robot does not accept the twist.
- The ranges of the twist sliders are conservative, for testing: `vx` ±0.2 m/s, `vy` ±0.1 m/s, `wz` ±0.2 rad/s.
  They are set in `mainwindow.h` (`max_vx_`, `max_vy_`, `max_wz_`).
- The legs of the B1 never accept targets: their sliders are disabled and follow the measured positions.
- A limb with its own driver (e.g. the Z1) does not know the mode of the base, so it only accepts targets while its
  *Allow commands* box is checked. Its sliders follow the measured positions until then, so the first drag starts
  from the current pose.
- `EMERGENCY STOP` sends the shutdown signal to the legged driver and to every limb with its own driver: their
  drivers stop the robots and exit.
- Closing the window (or `Ctrl + C` in its terminal) sends a zero twist.
