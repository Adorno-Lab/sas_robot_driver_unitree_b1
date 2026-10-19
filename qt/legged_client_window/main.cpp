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

#include "mainwindow.h"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

/**
 * Parameters (e.g. --ros-args -p legged_robot_prefix:=sas_b1/b1_2):
 *   - legged_robot_prefix (string, default "sas_b1/b1_1"): the legged robot driver.
 *   - own_driver_limb_prefixes (string array, default []): limbs with their own driver,
 *     e.g. "['sas_z1/z1_1']" for the Z1 arm of a B1.
 *   - monitor_only (bool, default false): only read the robot. The command topics are not created, so
 *     the window can run next to the controller of the robot. The emergency stop still works.
 */
int main(int argc, char *argv[])
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    // Keep our SIGINT handler: it closes the window, which stops the twist.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_legged_client_window");

    ClientWindowConfiguration configuration;
    configuration.legged_robot_prefix = node->declare_parameter<std::string>("legged_robot_prefix", "sas_b1/b1_1");
    configuration.own_driver_limb_prefixes = node->declare_parameter<std::vector<std::string>>("own_driver_limb_prefixes",
                                                                                               std::vector<std::string>{});
    configuration.monitor_only = node->declare_parameter<bool>("monitor_only", false);

    std::string own_driver_limb_prefixes;
    for (const auto& prefix : configuration.own_driver_limb_prefixes)
        own_driver_limb_prefixes += prefix + " ";
    RCLCPP_INFO_STREAM(node->get_logger(), "::legged_robot_prefix " << configuration.legged_robot_prefix
                                           << ", own_driver_limb_prefixes [ " << own_driver_limb_prefixes << "]"
                                           << ", monitor_only " << configuration.monitor_only);

    int result = 0;
    {
        QApplication application(argc, argv);
        MainWindow window{node, configuration, &kill_this_process};
        window.show();
        result = application.exec();
    }
    rclcpp::shutdown();
    return result;
}
