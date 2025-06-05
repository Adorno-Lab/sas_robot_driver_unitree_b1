![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)


# sas_robot_driver_unitree_b1

### Docker Instructions

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
> The robot will move with the next command. Be ready to perform an emergency stop!

```shell
start_ROS_drivers
```
