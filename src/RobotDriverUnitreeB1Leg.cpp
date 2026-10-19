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

#include "RobotDriverUnitreeB1Leg.h"
#include <stdexcept>

namespace sas
{

RobotDriverUnitreeB1Leg::RobotDriverUnitreeB1Leg(const std::shared_ptr<DriverUnitreeB1> &hardware,
                                                 const DriverUnitreeB1::BRANCH &branch,
                                                 const std::tuple<Eigen::VectorXd, Eigen::VectorXd> &joint_limits,
                                                 const std::shared_ptr<ShutdownSignaler> &shutdown_signaler):
    RobotDriver(shutdown_signaler),
    hardware_(hardware),
    branch_(branch)
{
    set_joint_limits(joint_limits);
}

Eigen::VectorXd RobotDriverUnitreeB1Leg::get_joint_positions()
{
    return hardware_->get_joint_positions(branch_);
}

Eigen::VectorXd RobotDriverUnitreeB1Leg::get_joint_velocities()
{
    return hardware_->get_joint_velocities(branch_);
}

Eigen::VectorXd RobotDriverUnitreeB1Leg::get_joint_torques()
{
    return hardware_->get_joint_estimated_torques(branch_);
}

void RobotDriverUnitreeB1Leg::set_target_joint_positions([[maybe_unused]] const Eigen::VectorXd &target_joint_positions)
{
    throw std::logic_error("RobotDriverUnitreeB1Leg::set_target_joint_positions: the legs are moved by the "
                           "locomotion controller and cannot be commanded in high-level control.");
}

// The lifecycle of the hardware belongs to RobotDriverUnitreeB1.
void RobotDriverUnitreeB1Leg::connect()
{

}

void RobotDriverUnitreeB1Leg::disconnect()
{

}

void RobotDriverUnitreeB1Leg::initialize()
{

}

void RobotDriverUnitreeB1Leg::deinitialize()
{

}

}
