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

#include "unitree_b1_node.hpp"

#include "DriverUnitreeB1.hpp"
#include <iostream>
#include <memory>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>

//using std::placeholders::_1;

namespace sas
{

class RobotDriverUnitreeB1::Impl
{

public:
   std::shared_ptr<DriverUnitreeB1> unitree_b1_driver_;
   Impl()
   {

   };



};

RobotDriverUnitreeB1::RobotDriverUnitreeB1(std::shared_ptr<Node> &node,
                                           const RobotDriverUnitreeB1Configuration &configuration,
                                           std::atomic_bool *break_loops)
    :st_break_loops_{break_loops},
    topic_prefix_{configuration.robot_name},
    configuration_{configuration},node_{node},
    timer_period_{0.002}, print_count_{0},
    clock_{0.002},
    watchdog_started_{false}
{
    impl_ = std::make_unique<RobotDriverUnitreeB1::Impl>();

    DriverUnitreeB1::MODE mode;
    if (configuration.mode == "VelocityControl")
    {
        mode = DriverUnitreeB1::MODE::VelocityControl;
    }else{
        mode = DriverUnitreeB1::MODE::None;
    }

    // I need to use the parameters of the configuration structure!
    impl_->unitree_b1_driver_ = std::make_shared<DriverUnitreeB1>(break_loops,
                                                                  mode, // Driver mode
                                                                  DriverUnitreeB1::LEVEL::HIGH,       // Level mode
                                                                  true,   //verbosity
                                                                  2000,   // TIMEOUT in ms
                                                                  configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE, // LIE DOWN ROBOT WHEN DEINITIALIZE
                                                                  configuration_.ROBOT_IP,  // Target IP   //192.168.123.10 for low-level mode
                                                                  configuration_.ROBOT_PORT,// Target port  //8007 for low-level mode
                                                                  8090);







    publisher_FR_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/FR_joint_states",1);
    publisher_FL_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/FL_joint_states",1);
    publisher_RR_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/RR_joint_states",1);
    publisher_RL_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix_ + "/get/RL_joint_states",1);


    publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_prefix_ + "/get/IMU_state", 1);
    publisher_pose_state_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix_ + "/get/pose_state", 1);
    publisher_high_level_velocities_state_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_prefix_ + "/get/twist_state", 1);

    publisher_battery_state_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(topic_prefix_ + "/get/battery_state", 1);



    subscriber_target_holonomic_velocities_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_prefix_ + "/set/holonomic_target_velocities",
        1,
        std::bind(&RobotDriverUnitreeB1::_callback_target_holonomic_velocities, this, std::placeholders::_1)
        );


    subscriber_watchdog_trigger_ = node_->create_subscription<sas_msgs::msg::WatchdogTrigger>(
        topic_prefix_ + "/set/watchdog_trigger", 1,
        std::bind(&RobotDriverUnitreeB1::_callback_watchdog_trigger_state, this, std::placeholders::_1)
        );


}

void RobotDriverUnitreeB1::_read_joint_states_and_publish()
{
    sensor_msgs::msg::JointState ros_msg_FR;
    sensor_msgs::msg::JointState ros_msg_FL;
    sensor_msgs::msg::JointState ros_msg_RR;
    sensor_msgs::msg::JointState ros_msg_RL;

    ros_msg_FR.header.stamp =  node_->get_clock()->now();
    ros_msg_FL.header.stamp =  node_->get_clock()->now();
    ros_msg_RR.header.stamp =  node_->get_clock()->now();
    ros_msg_RL.header.stamp =  node_->get_clock()->now();

    VectorXd qFR = impl_->unitree_b1_driver_->get_joint_positions(DriverUnitreeB1::BRANCH::FR);
    VectorXd qFR_dot = impl_->unitree_b1_driver_->get_joint_velocities(DriverUnitreeB1::BRANCH::FR);
    VectorXd qFR_tau = impl_->unitree_b1_driver_->get_joint_estimated_torques(DriverUnitreeB1::BRANCH::FR);

    VectorXd qFL = impl_->unitree_b1_driver_->get_joint_positions(DriverUnitreeB1::BRANCH::FL);
    VectorXd qFL_dot = impl_->unitree_b1_driver_->get_joint_velocities(DriverUnitreeB1::BRANCH::FL);
    VectorXd qFL_tau = impl_->unitree_b1_driver_->get_joint_estimated_torques(DriverUnitreeB1::BRANCH::FL);

    VectorXd qRR = impl_->unitree_b1_driver_->get_joint_positions(DriverUnitreeB1::BRANCH::RR);
    VectorXd qRR_dot = impl_->unitree_b1_driver_->get_joint_velocities(DriverUnitreeB1::BRANCH::RR);
    VectorXd qRR_tau = impl_->unitree_b1_driver_->get_joint_estimated_torques(DriverUnitreeB1::BRANCH::RR);

    VectorXd qRL = impl_->unitree_b1_driver_->get_joint_positions(DriverUnitreeB1::BRANCH::RL);
    VectorXd qRL_dot = impl_->unitree_b1_driver_->get_joint_velocities(DriverUnitreeB1::BRANCH::RL);
    VectorXd qRL_tau = impl_->unitree_b1_driver_->get_joint_estimated_torques(DriverUnitreeB1::BRANCH::RL);

    if (qFR.size() > 0)
        ros_msg_FR.position = vectorxd_to_std_vector_double(qFR);
    if (qFR_dot.size() > 0)
        ros_msg_FR.velocity = vectorxd_to_std_vector_double(qFR_dot);
    if (qFR_tau.size() > 0)
        ros_msg_FR.effort = vectorxd_to_std_vector_double(qFR_tau);

    if (qFL.size() > 0)
        ros_msg_FL.position = vectorxd_to_std_vector_double(qFL);
    if (qFL_dot.size() > 0)
        ros_msg_FL.velocity = vectorxd_to_std_vector_double(qFL_dot);
    if (qFL_tau.size() > 0)
        ros_msg_FL.effort = vectorxd_to_std_vector_double(qFL_tau);

    if (qRR.size() > 0)
        ros_msg_RR.position = vectorxd_to_std_vector_double(qRR);
    if (qRR_dot.size() > 0)
        ros_msg_RR.velocity = vectorxd_to_std_vector_double(qRR_dot);
    if (qRR_tau.size() > 0)
        ros_msg_RR.effort = vectorxd_to_std_vector_double(qRR_tau);


    if (qRL.size() > 0)
        ros_msg_RL.position = vectorxd_to_std_vector_double(qRL);
    if (qRL_dot.size() > 0)
        ros_msg_RL.velocity = vectorxd_to_std_vector_double(qRL_dot);
    if (qRL_tau.size() > 0)
        ros_msg_RL.effort = vectorxd_to_std_vector_double(qRL_tau);

    publisher_FR_joint_states_->publish(ros_msg_FR);
    publisher_FL_joint_states_->publish(ros_msg_FL);
    publisher_RR_joint_states_->publish(ros_msg_RR);
    publisher_RL_joint_states_->publish(ros_msg_RL);

}

void RobotDriverUnitreeB1::_read_imu_state_and_publish()
{
    sensor_msgs::msg::Imu ros_msg_imu;
    ros_msg_imu.header.stamp = node_->get_clock()->now();

    geometry_msgs::msg::PoseStamped ros_msg_pose;
    ros_msg_pose.header.stamp = node_->get_clock()->now();

    VectorXd vec_orientation = impl_->unitree_b1_driver_->get_IMU_orientation().vec4();
    ros_msg_imu.orientation.w = vec_orientation(0);
    ros_msg_imu.orientation.x = vec_orientation(1);
    ros_msg_imu.orientation.y = vec_orientation(2);
    ros_msg_imu.orientation.z = vec_orientation(3);

    VectorXd vec_angular_velocity = impl_->unitree_b1_driver_->get_IMU_gyroscope().vec3();
    ros_msg_imu.angular_velocity.x = vec_angular_velocity(0);
    ros_msg_imu.angular_velocity.y = vec_angular_velocity(1);
    ros_msg_imu.angular_velocity.z = vec_angular_velocity(2);

    VectorXd vec_acceleration = impl_->unitree_b1_driver_->get_IMU_accelerometer().vec3();
    ros_msg_imu.linear_acceleration.x = vec_acceleration(0);
    ros_msg_imu.linear_acceleration.y = vec_acceleration(1);
    ros_msg_imu.linear_acceleration.z = vec_acceleration(2);

    VectorXd vec_position = impl_->unitree_b1_driver_->get_IMU_pose().translation().vec3();
    ros_msg_pose.pose.position.x = vec_position(0);
    ros_msg_pose.pose.position.y = vec_position(1);
    ros_msg_pose.pose.position.z = vec_position(2);

    ros_msg_pose.pose.orientation.w = vec_orientation(0);
    ros_msg_pose.pose.orientation.x = vec_orientation(1);
    ros_msg_pose.pose.orientation.y = vec_orientation(2);
    ros_msg_pose.pose.orientation.z = vec_orientation(3);

    publisher_IMU_state_->publish(ros_msg_imu);
    publisher_pose_state_->publish(ros_msg_pose);
}

void RobotDriverUnitreeB1::_read_twist_state_and_publish()
{
    geometry_msgs::msg::TwistStamped ros_msg_twist;
    ros_msg_twist.header.stamp = node_->get_clock()->now();

    VectorXd vec_angular_velocity = impl_->unitree_b1_driver_->get_high_level_angular_velocity().vec3();
    ros_msg_twist.twist.angular.x = vec_angular_velocity(0);
    ros_msg_twist.twist.angular.y = vec_angular_velocity(1);
    ros_msg_twist.twist.angular.z = vec_angular_velocity(2);

    VectorXd vec_acceleration = impl_->unitree_b1_driver_->get_high_level_linear_velocity().vec3();
    ros_msg_twist.twist.linear.x = vec_acceleration(0);
    ros_msg_twist.twist.linear.y = vec_acceleration(1);
    ros_msg_twist.twist.linear.z = vec_acceleration(2);

    publisher_high_level_velocities_state_->publish(ros_msg_twist);

}

void RobotDriverUnitreeB1::_read_battery_state()
{
    sensor_msgs::msg::BatteryState ros_msg_battery;
    ros_msg_battery.header.stamp = node_->get_clock()->now();
    int battery_level = impl_->unitree_b1_driver_->get_state_of_charge();

    ros_msg_battery.percentage = 0.01*battery_level;
    publisher_battery_state_->publish(ros_msg_battery);
}



bool RobotDriverUnitreeB1::_should_shutdown() const
{
    return (*st_break_loops_);
}

void RobotDriverUnitreeB1::_set_target_velocities_from_subscriber()
{
    if (new_target_velocities_available_)
    {
        impl_->unitree_b1_driver_->set_high_level_speed(target_holonomic_velocities_(0),
                                                        target_holonomic_velocities_(1),
                                                        target_holonomic_velocities_(2));
        new_target_velocities_available_ = false;
    }
}

void RobotDriverUnitreeB1::_callback_watchdog_trigger_state(const sas_msgs::msg::WatchdogTrigger& msg)
{
    std::scoped_lock lock(mutex_watchdog_);
    watchdog_enabled_ = true;
    watchdog_trigger_status_ = msg.status;
    last_trigger_ = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::seconds(msg.header.stamp.sec) + std::chrono::nanoseconds(msg.header.stamp.nanosec)
        );
}

bool RobotDriverUnitreeB1::is_watchdog_enabled() const
{
    return watchdog_enabled_;
}



void RobotDriverUnitreeB1::control_loop()
{
    try{

        clock_.init();
        impl_->unitree_b1_driver_->connect();
        impl_->unitree_b1_driver_->initialize();



        while(not _should_shutdown())
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);

            _read_joint_states_and_publish();
            _read_imu_state_and_publish();
            _read_battery_state();
            _read_twist_state_and_publish();
            _set_target_velocities_from_subscriber();

            if (is_watchdog_enabled())
            {
                if (!watchdog_started_)
                {   // This portion of code is executed only one time
                    // Initialize the watchdog.
                    double watchdog_period;
                    // If the "watchdog_period_in_seconds" is not defined, we use a default value.
                    get_ros_optional_parameter(node_, "watchdog_period_in_seconds", watchdog_period, 1.0);
                    RCLCPP_INFO_STREAM(node_->get_logger(), "Watchdog initialized with a " << watchdog_period << " second period");
                    const std::chrono::nanoseconds period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(watchdog_period));
                    watchdog_started_ = true;
                    _watchdog_start(period);
                }

            }

            rclcpp::spin_some(node_);
        }
        impl_->unitree_b1_driver_->set_high_level_speed(0,0,0);
    }
    catch(const std::exception& e)
    {
        std::cerr<<"::Exception caught::" << e.what() <<std::endl;
    }
}

void RobotDriverUnitreeB1::_callback_target_holonomic_velocities(const std_msgs::msg::Float64MultiArray &msg)
{
    target_holonomic_velocities_  = std_vector_double_to_vectorxd(msg.data);
    new_target_velocities_available_ = true;
}

void RobotDriverUnitreeB1::_watchdog_start(const std::chrono::nanoseconds& period)
{
    if (!watchdog_clock_)
        watchdog_clock_ = std::make_unique<sas::Clock>(std::chrono::duration_cast<std::chrono::duration<double>>(period).count());

    if (!watchdog_thread_)
        watchdog_thread_ = std::make_unique<std::thread>(&RobotDriverUnitreeB1::_watchdog_thread_function, this);
}


void RobotDriverUnitreeB1::_watchdog_thread_function()
{
    const double& period =  watchdog_clock_->get_desired_thread_sampling_time_sec();
    watchdog_clock_->init();
    while(!_should_shutdown())
    {
        std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
        double elapsed_time;
        bool wstatus;
        {
            std::scoped_lock lock(mutex_watchdog_);
            elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - last_trigger_).count();
            wstatus = watchdog_status_;
        }
        if (elapsed_time > period)
            throw std::runtime_error(
                std::string("RobotDriverUnitreeB1:: The watchdog signal was lost! ") +
                "The elapsed time was " + std::to_string(elapsed_time) +
                " but the period is: " + std::to_string(period)
                );

        if(!wstatus)
            throw std::runtime_error("RobotDriverUnitreeB1:: The watchdog status is false!");
        watchdog_clock_->update_and_sleep();
    }
}


RobotDriverUnitreeB1::~RobotDriverUnitreeB1()
{
    impl_->unitree_b1_driver_->deinitialize();
    impl_->unitree_b1_driver_->disconnect();
}


}
