/*
# (C) Copyright 2024-2025 Adorno-Lab software developments
#
#    This file is part of sas_robot_driver_unitree_b1.
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on https://ros2-tutorial.readthedocs.io/en/latest/cpp/cpp_node.html
#
# ################################################################*/

#pragma once
#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_msgs/msg/watchdog_trigger.hpp>
#include <sas_msgs/msg/bool.hpp>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>
//using namespace Eigen;

using namespace rclcpp;

namespace sas
{

struct RobotDriverUnitreeB1Configuration
{
    std::string mode;              //const std::string mode= "PositionControl";
    bool LIE_DOWN_ROBOT_WHEN_DEINITIALIZE;  //std::string LIE_DOWN_ROBOT_WHEN_DEINITIALIZE
    std::string ROBOT_IP; //"192.168.123.220",  // Target IP   //192.168.123.10 for low-level mode
    int ROBOT_PORT; //    8082,              // Target port  //8007 for low-level mode
    std::string robot_name;
    //double watchdog_period_in_seconds;
    bool FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO; // to handle this https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/issues/4
};




class RobotDriverUnitreeB1: public LeggedRobotDriver
{
protected:
    std::atomic_bool* st_break_loops_;
    std::string topic_prefix_;
    RobotDriverUnitreeB1Configuration configuration_;
    std::shared_ptr<rclcpp::Node> node_;

private:
    double timer_period_;

    int print_count_;

    sas::Clock clock_;


    //also equivalent to rclcpp::TimerBase::SharedPtr
    std::shared_ptr<rclcpp::TimerBase> timer_;

    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_joint_positions_;

    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_FR_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_FL_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_RR_joint_states_;
    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_RL_joint_states_;

    Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU_state_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_state_;
    Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_battery_state_;
    Publisher<geometry_msgs::msg::TwistStamped>:: SharedPtr publisher_high_level_velocities_state_;

    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_target_holonomic_velocities_;

    VectorXd target_holonomic_velocities_ = VectorXd::Zero(3);
    void _callback_target_holonomic_velocities(const std_msgs::msg::Float64MultiArray& msg);
    bool new_target_velocities_available_{false};

    Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_shutdown_signal_;
    void _callback_shutdown_signal_(const sas_msgs::msg::Bool& msg);
    bool shutdown_signal_;


    Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_emergency_stop_device_signal_;
    void _callback_emergency_stop_device_signal_(const sas_msgs::msg::Bool& msg);



    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;

    void _initial_settings();

protected:

    void _read_joint_states_and_publish();
    void _read_imu_state_and_publish();
    void _read_twist_state_and_publish();
    void _read_battery_state();
    bool _should_shutdown() const;
    void _set_target_velocities_from_subscriber();

    void _watchdog_set_maximum_acceptable_delay(const double& max_acceptable_delay);

public:

    RobotDriverUnitreeB1(const RobotDriverUnitreeB1&)=delete;
    RobotDriverUnitreeB1()=delete;
    ~RobotDriverUnitreeB1();

    RobotDriverUnitreeB1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeB1Configuration &configuration,
                         std::atomic_bool* break_loops);

    RobotDriverUnitreeB1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeB1Configuration &configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    //void control_loop();


    //----------RobotDriver----methods---------------------------------//

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    //void set_target_joint_velocities(const VectorXd& desired_joint_velocities_rad_s) override;

    VectorXd get_joint_velocities() override;
    VectorXd get_joint_torques() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    void set_target_twist(const DQ& twist) override;

    void set_target_base_orientation(const DQ& r) override;

    void set_target_base_height(const double& base_height) override;



};



}
