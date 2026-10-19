/*
# (C) Copyright 2024-2026 Adorno-Lab software developments
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
#
# ################################################################*/

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_tools/sas_legged_robot_driver_ros.hpp>
#include <sas_robot_driver_unitree_b1/sas_robot_driver_unitree_b1.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler = std::make_shared<sas::ShutdownSignaler>();
void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

/**
 * Parameters (set by the launch file):
 *   - ROBOT_IP (string, mandatory): address of the robot, e.g. "192.168.123.220".
 *   - ROBOT_PORT (int, mandatory): port of the high-level controller of the robot, e.g. 8082.
 *   - LIE_DOWN_ROBOT_WHEN_DEINITIALIZE (bool, mandatory): lie the robot down (instead of a quick stop) when
 *     the driver stops.
 *   - thread_sampling_time_sec (double, mandatory): period of the control loop, e.g. 0.002.
 *   - mode (string, optional, default "VelocityControl"): "VelocityControl" (the robot walks with the target
 *     twist) or "None" (the driver only reads the state of the robot).
 *   - LOCAL_PORT (int, optional, default 8090): port of this computer.
 *   - FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO (bool, optional, default false): see
 *     https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/issues/4
 *   - twist_timeout_sec (double, optional, default 0.2): a zero twist is sent when no twist arrives in time.
 *
 * The topic prefix is the name of the node (set by the launch file), inside its namespace, e.g. the node
 * "b1_1" in the namespace "sas_b1" serves sas_b1/b1_1/... as in the other SAS robot drivers.
 */
int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    // Keep our SIGINT handler: it signals the shutdown so that the control loop ends and the driver
    // deinitializes the robot before ROS is shut down.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_b1");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverUnitreeB1Configuration robot_configuration;
        sas::get_ros_parameter(node, "ROBOT_IP", robot_configuration.robot_ip);
        sas::get_ros_parameter(node, "ROBOT_PORT", robot_configuration.robot_port);
        sas::get_ros_parameter(node, "LIE_DOWN_ROBOT_WHEN_DEINITIALIZE", robot_configuration.lie_down_robot_when_deinitialize);
        sas::get_ros_optional_parameter(node, "LOCAL_PORT", robot_configuration.local_port, 8090);
        sas::get_ros_optional_parameter(node, "FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO",
                                        robot_configuration.force_stand_mode_when_high_level_velocities_are_zero, false);

        std::string mode;
        sas::get_ros_optional_parameter(node, "mode", mode, std::string("VelocityControl"));
        if (mode == "VelocityControl")
            robot_configuration.mode = sas::RobotDriverUnitreeB1Configuration::MODE::VELOCITY_CONTROL;
        else if (mode == "None")
            robot_configuration.mode = sas::RobotDriverUnitreeB1Configuration::MODE::NONE;
        else
            throw std::invalid_argument("The parameter mode must be \"VelocityControl\" or \"None\", but it is \"" +
                                        mode + "\".");

        sas::LeggedRobotDriverROSConfiguration configuration{};
        configuration.robot_driver_ros.robot_driver_provider_prefix = node->get_name();
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.robot_driver_ros.thread_sampling_time_sec);
        sas::get_ros_optional_parameter(node, "twist_timeout_sec", configuration.twist_timeout_sec, 0.2);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK: mode " << mode
                                                    << ", ROBOT_IP " << robot_configuration.robot_ip
                                                    << ", ROBOT_PORT " << robot_configuration.robot_port
                                                    << ", LOCAL_PORT " << robot_configuration.local_port
                                                    << ", LIE_DOWN_ROBOT_WHEN_DEINITIALIZE " << robot_configuration.lie_down_robot_when_deinitialize
                                                    << ", FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO " << robot_configuration.force_stand_mode_when_high_level_velocities_are_zero
                                                    << ", prefix " << node->get_fully_qualified_name()
                                                    << ", thread_sampling_time_sec " << configuration.robot_driver_ros.thread_sampling_time_sec
                                                    << ", twist_timeout_sec " << configuration.twist_timeout_sec);

        auto robot_driver = std::make_shared<sas::RobotDriverUnitreeB1>(robot_configuration, shutdown_signaler);

        sas::LeggedRobotDriverROS legged_robot_driver_ros(node, robot_driver, configuration, shutdown_signaler);
        legged_robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    rclcpp::shutdown();
    return 0;
}
