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
#include <optional>

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
    double watchdog_period_in_seconds;
};




class RobotDriverUnitreeB1
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


    //------Watchdog-----------------------------------------------------------------
    Subscription<sas_msgs::msg::WatchdogTrigger>::SharedPtr subscriber_watchdog_trigger_;
    void _callback_watchdog_trigger_state(const sas_msgs::msg::WatchdogTrigger& msg);
    bool watchdog_trigger_status_;
    bool watchdog_enabled_;
    bool is_watchdog_enabled() const;
    bool watchdog_started_;
    std::unique_ptr<sas::Clock> watchdog_clock_;
    std::unique_ptr<std::thread> watchdog_thread_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> last_trigger_;
    bool watchdog_status_;
    void _watchdog_thread_function();
    void _watchdog_start(const std::chrono::nanoseconds& period);


    std::mutex mutex_watchdog_;


    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;

protected:

    void _read_joint_states_and_publish();
    void _read_imu_state_and_publish();
    void _read_twist_state_and_publish();
    void _read_battery_state();
    bool _should_shutdown() const;
    void _set_target_velocities_from_subscriber();



public:

    RobotDriverUnitreeB1(const RobotDriverUnitreeB1&)=delete;
    RobotDriverUnitreeB1()=delete;
    ~RobotDriverUnitreeB1();

    RobotDriverUnitreeB1(std::shared_ptr<Node>& node,
                         const RobotDriverUnitreeB1Configuration &configuration,
                         std::atomic_bool* break_loops);

    void control_loop();

};



}
