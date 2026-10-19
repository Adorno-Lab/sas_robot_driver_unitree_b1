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
#include <string>
#include <vector>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_tools/LeggedRobotDriver.hpp>

namespace sas
{

struct RobotDriverUnitreeB1Configuration
{
    /**
     * @brief What the driver does with the robot.
     *   - VELOCITY_CONTROL: the robot walks with the target twist (high-level control).
     *   - NONE: the driver only reads the state of the robot. It never commands it.
     */
    enum class MODE{
        VELOCITY_CONTROL=0,
        NONE,
    };

    MODE mode{MODE::VELOCITY_CONTROL};
    std::string robot_ip;      ///< e.g. "192.168.123.220" (LAN) or the address of the robot in the WiFi network.
    int robot_port{8082};      ///< Port of the high-level controller of the robot.
    int local_port{8090};      ///< Port of this computer.
    bool lie_down_robot_when_deinitialize{false};  ///< Lie the robot down (instead of a quick stop) in deinitialize().
    bool force_stand_mode_when_high_level_velocities_are_zero{false}; ///< See https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1/issues/4
};

/**
 * @brief The RobotDriverUnitreeB1 class is the LeggedRobotDriver of the Unitree B1. It does not use ROS;
 *        run it with LeggedRobotDriverROS.
 *
 * The robot is commanded in high-level control (UDP, unitree_legged_sdk): Unitree's locomotion controller
 * moves the legs with the target twist.
 *
 * Limbs (served on \<prefix\>/\<name\>), with the joint order of unitree_legged_sdk (hip, thigh, calf):
 *   - front_right_leg, front_left_leg, rear_right_leg, rear_left_leg: never commandable (moved by the
 *     locomotion controller).
 *
 * Modes:
 *   - IDLE: zero velocity.
 *   - WALKING: the twist is accepted (MODE::VELOCITY_CONTROL only).
 *
 * Every mode change first stops the robot (zero velocity). Only the twist is supported: neither the base
 * height nor the base orientation can be commanded, so STANDING is not supported.
 */
class RobotDriverUnitreeB1: public LeggedRobotDriver
{
private:
    class Impl;
    std::unique_ptr<Impl> impl_;

protected:
    void _set_high_level_mode(const HIGH_LEVEL_MODE& mode) override;

public:
    ~RobotDriverUnitreeB1();
    RobotDriverUnitreeB1()=delete;
    RobotDriverUnitreeB1(const RobotDriverUnitreeB1&) = delete;
    RobotDriverUnitreeB1& operator=(const RobotDriverUnitreeB1&) = delete;
    RobotDriverUnitreeB1(RobotDriverUnitreeB1&&) = delete;
    RobotDriverUnitreeB1& operator=(RobotDriverUnitreeB1&&) = delete;

    /**
     * @brief RobotDriverUnitreeB1 Creates the driver and its four legs. It does not connect yet.
     * @param configuration The mode, the network settings, and the stop behaviour.
     * @param shutdown_signaler Shared with LeggedRobotDriverROS.
     */
    RobotDriverUnitreeB1(const RobotDriverUnitreeB1Configuration& configuration,
                         const std::shared_ptr<ShutdownSignaler>& shutdown_signaler);

    /**
     * @brief connect Starts the UDP communication with the robot.
     * @throws std::runtime_error if the robot does not answer within 2 s.
     */
    void connect() override;
    void disconnect() override;

    /**
     * @brief initialize Starts commanding the robot (MODE::VELOCITY_CONTROL), with zero velocity.
     */
    void initialize() override;

    /**
     * @brief deinitialize Stops the robot (a quick stop, or lying down; see
     *        RobotDriverUnitreeB1Configuration) and the communication.
     */
    void deinitialize() override;

    /**
     * @brief set_target_twist Sets the target velocities of the locomotion controller.
     * @param twist Only wz (yaw rate), vx, and vy are used, in the body frame. They are saturated to the
     *        ranges of the robot: vx [-0.8, 1.2] m/s, vy [-0.25, 0.25] m/s, wz [-0.75, 0.75] rad/s.
     * @throws std::invalid_argument if a velocity is not finite. The robot is commanded to stop.
     */
    void set_target_twist(const DQ& twist) override;

    /**
     * @brief set_target_base_orientation Not supported by the B1 driver.
     * @throws std::logic_error always (LeggedRobotDriverROS never calls it, since is_supported() is false).
     */
    void set_target_base_orientation(const DQ& r) override;

    /**
     * @brief set_target_base_height Not supported by the B1 driver.
     * @throws std::logic_error always (LeggedRobotDriverROS never calls it, since is_supported() is false).
     */
    void set_target_base_height(const double& base_height) override;

    /**
     * @brief get_orientation Returns the IMU orientation.
     * @return A unit quaternion, or DQ(0) until the first message of the robot arrives.
     */
    DQ get_orientation() override;

    /**
     * @brief get_angular_velocity Returns the IMU angular velocity, in rad/s, in the body frame.
     * @return A pure quaternion. Zero until the first message of the robot arrives.
     */
    DQ get_angular_velocity() override;

    /**
     * @brief get_linear_acceleration Returns the IMU linear acceleration (with gravity), in m/s^2,
     *        in the body frame.
     * @return A pure quaternion. Zero until the first message of the robot arrives.
     */
    DQ get_linear_acceleration() override;

    /**
     * @brief get_supported_high_level_modes Returns IDLE and WALKING in MODE::VELOCITY_CONTROL, and IDLE
     *        in MODE::NONE.
     */
    std::vector<HIGH_LEVEL_MODE> get_supported_high_level_modes() const override;

    /**
     * @brief is_supported Returns true only for LEGGED_FUNCTIONALITY::TWIST in MODE::VELOCITY_CONTROL, and
     *        false for every functionality in MODE::NONE.
     */
    bool is_supported(const LEGGED_FUNCTIONALITY& functionality) const override;

    /**
     * @brief get_limbs Returns front_right_leg, front_left_leg, rear_right_leg, and rear_left_leg, in this order.
     */
    std::vector<LimbEntry> get_limbs() const override;

    /**
     * @brief get_commandable_limbs No leg accepts targets: they are moved by the locomotion controller.
     * @return One entry per limb of get_limbs(), all false.
     */
    std::vector<bool> get_commandable_limbs() const override;
};

}
