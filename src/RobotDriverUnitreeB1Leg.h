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

#pragma once
#include <memory>
#include <tuple>
#include <sas_core/sas_robot_driver.hpp>
#include "DriverUnitreeB1.hpp"

namespace sas
{

/**
 * @brief The RobotDriverUnitreeB1Leg class is a RobotDriver view of one leg of the Unitree B1, on top of
 *        the DriverUnitreeB1 shared by every leg.
 *
 * The joints of each leg are hip, thigh, and calf, in this order (unitree_legged_sdk). The lifecycle
 * (connect(), initialize(), ...) belongs to RobotDriverUnitreeB1, so it is a no-op here.
 */
class RobotDriverUnitreeB1Leg: public RobotDriver
{
private:
    std::shared_ptr<DriverUnitreeB1> hardware_;
    DriverUnitreeB1::BRANCH branch_;

public:
    RobotDriverUnitreeB1Leg()=delete;
    RobotDriverUnitreeB1Leg(const RobotDriverUnitreeB1Leg&)=delete;

    /**
     * @brief RobotDriverUnitreeB1Leg
     * @param hardware The driver of the B1, shared by every leg.
     * @param branch The leg of this view.
     * @param joint_limits {min, max} joint positions of the leg, in radians.
     * @param shutdown_signaler Shared with RobotDriverUnitreeB1.
     */
    RobotDriverUnitreeB1Leg(const std::shared_ptr<DriverUnitreeB1>& hardware,
                            const DriverUnitreeB1::BRANCH& branch,
                            const std::tuple<Eigen::VectorXd, Eigen::VectorXd>& joint_limits,
                            const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief get_joint_positions Returns the joint positions of the leg, in radians. Zero until the first
     *        message of the robot arrives.
     */
    Eigen::VectorXd get_joint_positions() override;

    /**
     * @brief get_joint_velocities Returns the joint velocities of the leg, in rad/s. Zero until the first
     *        message of the robot arrives.
     */
    Eigen::VectorXd get_joint_velocities() override;

    /**
     * @brief get_joint_torques Returns the estimated joint torques of the leg, in Nm. Zero until the first
     *        message of the robot arrives.
     */
    Eigen::VectorXd get_joint_torques() override;

    /**
     * @brief set_target_joint_positions The legs are moved by the locomotion controller.
     * @throws std::logic_error always (LeggedRobotDriverROS never calls it, since no leg is commandable).
     */
    void set_target_joint_positions(const Eigen::VectorXd& target_joint_positions) override;

    void connect() override;
    void disconnect() override;
    void initialize() override;
    void deinitialize() override;
};

}
