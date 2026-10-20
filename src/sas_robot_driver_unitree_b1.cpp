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

#include <sas_robot_driver_unitree_b1/sas_robot_driver_unitree_b1.hpp>
#include "DriverUnitreeB1.hpp"
#include "RobotDriverUnitreeB1Leg.h"
#include <sas_tools/rpy_conversions.hpp>
#include <cmath>
#include <stdexcept>

namespace sas
{

namespace
{

using BRANCH = DriverUnitreeB1::BRANCH;

/**
 * @brief One leg of the B1. The joint names are the ones of unitree_ros/robots/b1_description/xacro/b1.urdf,
 *        in the order of unitree_legged_sdk (hip, thigh, calf). The joint limits, in radians, are the ones of
 *        unitree_legged_sdk/b1_const.h (identical in b1.urdf), and are the same for every leg.
 */
struct B1Leg
{
    std::string name;
    BRANCH branch;
    std::vector<std::string> joint_names;
};

const std::vector<B1Leg> kB1Legs = {
    {"front_right_leg", BRANCH::FR, {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
    {"front_left_leg",  BRANCH::FL, {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
    {"rear_right_leg",  BRANCH::RR, {"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}},
    {"rear_left_leg",   BRANCH::RL, {"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}},
};

const VectorXd kLegLowerLimits = (VectorXd(3) << -0.75, -1.0, -2.6).finished();
const VectorXd kLegUpperLimits = (VectorXd(3) <<  0.75,  3.5, -0.6).finished();

// Ranges of the standing pose (roll, pitch, yaw), in radians, and nominal body height, in meters
// (unitree_legged_sdk/comm.h). The B1 takes the body height as a difference with the nominal one.
const Eigen::Vector3d kBaseOrientationLowerLimits(-0.3, -0.3, -0.6);
const Eigen::Vector3d kBaseOrientationUpperLimits( 0.3,  0.3,  0.6);
const double kNominalBaseHeight = 0.58;
const double kMaxBaseHeightDelta = 0.16;

}

class RobotDriverUnitreeB1::Impl
{
public:
    RobotDriverUnitreeB1Configuration::MODE mode_;
    std::shared_ptr<DriverUnitreeB1> hardware_;
    std::vector<LimbEntry> limbs_;
};

RobotDriverUnitreeB1::~RobotDriverUnitreeB1()=default;

RobotDriverUnitreeB1::RobotDriverUnitreeB1(const RobotDriverUnitreeB1Configuration &configuration,
                                           const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :LeggedRobotDriver{shutdown_signaler},
    impl_{std::make_unique<RobotDriverUnitreeB1::Impl>()}
{
    impl_->mode_ = configuration.mode;

    std::vector<DriverUnitreeB1::CUSTOM_FLAGS> custom_flags;
    if (configuration.force_stand_mode_when_high_level_velocities_are_zero)
        custom_flags.push_back(DriverUnitreeB1::CUSTOM_FLAGS::FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO);

    impl_->hardware_ = std::make_shared<DriverUnitreeB1>(
        shutdown_signaler,
        impl_->mode_ == RobotDriverUnitreeB1Configuration::MODE::VELOCITY_CONTROL ? DriverUnitreeB1::MODE::VelocityControl
                                                                                   : DriverUnitreeB1::MODE::None,
        DriverUnitreeB1::LEVEL::HIGH,
        true,   // verbosity
        2000,   // timeout to connect, in ms
        configuration.lie_down_robot_when_deinitialize,
        configuration.robot_ip,
        configuration.robot_port,
        configuration.local_port,
        custom_flags);

    for (const auto& b1_leg : kB1Legs)
    {
        auto driver = std::make_shared<RobotDriverUnitreeB1Leg>(impl_->hardware_,
                                                                b1_leg.branch,
                                                                std::make_tuple(kLegLowerLimits, kLegUpperLimits),
                                                                shutdown_signaler);
        impl_->limbs_.push_back({b1_leg.name, driver, b1_leg.joint_names});
    }
}

void RobotDriverUnitreeB1::connect()
{
    impl_->hardware_->connect();
}

void RobotDriverUnitreeB1::disconnect()
{
    impl_->hardware_->disconnect();
}

void RobotDriverUnitreeB1::initialize()
{
    impl_->hardware_->initialize();
}

void RobotDriverUnitreeB1::deinitialize()
{
    impl_->hardware_->deinitialize();
}

void RobotDriverUnitreeB1::_set_high_level_mode(const HIGH_LEVEL_MODE &mode)
{
    // Stop before any mode change. In IDLE, LeggedRobotDriverROS keeps sending a zero twist; in WALKING,
    // the robot walks with the target twist. A change of the target behavior also resets the target pose, and
    // DriverUnitreeB1 handles the transitions (stop before standing, neutral pose before walking).
    impl_->hardware_->set_high_level_speed(0.0, 0.0, 0.0);
    impl_->hardware_->set_target_behavior(mode == HIGH_LEVEL_MODE::STANDING ? DriverUnitreeB1::TARGET_BEHAVIOR::STAND
                                                                            : DriverUnitreeB1::TARGET_BEHAVIOR::VELOCITY);
}

void RobotDriverUnitreeB1::set_target_twist(const DQ &twist)
{
    //    0  1  2  3  4  5
    //   wx wy wz vx vy vz
    const VectorXd twist_vec = twist.vec6();
    impl_->hardware_->set_high_level_speed(twist_vec(3), twist_vec(4), twist_vec(2));
}

void RobotDriverUnitreeB1::set_target_base_orientation(const DQ &r)
{
    if (!is_unit(r) || r.vec4().array().isNaN().any())
    {
        impl_->hardware_->set_high_level_stand_euler_angles(0.0, 0.0, 0.0);
        throw std::invalid_argument("RobotDriverUnitreeB1::set_target_base_orientation: the orientation must be a "
                                    "unit quaternion. The pose is reset to neutral.");
    }
    const Eigen::Vector3d rpy = unit_quaternion_to_rpy(r)
                                    .cwiseMax(kBaseOrientationLowerLimits)
                                    .cwiseMin(kBaseOrientationUpperLimits);
    impl_->hardware_->set_high_level_stand_euler_angles(rpy(0), rpy(1), rpy(2));
}

void RobotDriverUnitreeB1::set_target_base_height(const double &base_height)
{
    if (!std::isfinite(base_height))
    {
        impl_->hardware_->set_high_level_stand_body_height(0.0);
        throw std::invalid_argument("RobotDriverUnitreeB1::set_target_base_height: the height must be finite. "
                                    "The height is reset to the nominal one.");
    }
    // Saturated by DriverUnitreeB1 to [-0.16, 0.16] m.
    impl_->hardware_->set_high_level_stand_body_height(base_height - kNominalBaseHeight);
}

DQ RobotDriverUnitreeB1::get_orientation()
{
    return impl_->hardware_->get_IMU_orientation();
}

DQ RobotDriverUnitreeB1::get_angular_velocity()
{
    return impl_->hardware_->get_IMU_gyroscope();
}

DQ RobotDriverUnitreeB1::get_linear_acceleration()
{
    return impl_->hardware_->get_IMU_accelerometer();
}

std::vector<LeggedRobotDriver::HIGH_LEVEL_MODE> RobotDriverUnitreeB1::get_supported_high_level_modes() const
{
    if (impl_->mode_ == RobotDriverUnitreeB1Configuration::MODE::NONE)
        return {HIGH_LEVEL_MODE::IDLE};  // The robot is never commanded.
    return {HIGH_LEVEL_MODE::IDLE, HIGH_LEVEL_MODE::STANDING, HIGH_LEVEL_MODE::WALKING};
}

bool RobotDriverUnitreeB1::is_supported(const LEGGED_FUNCTIONALITY &functionality) const
{
    if (impl_->mode_ == RobotDriverUnitreeB1Configuration::MODE::NONE)
        return false;
    switch (functionality)
    {
    case LEGGED_FUNCTIONALITY::TWIST:
    case LEGGED_FUNCTIONALITY::BASE_HEIGHT:
    case LEGGED_FUNCTIONALITY::BASE_ORIENTATION:
        return true;
    case LEGGED_FUNCTIONALITY::BASE_HEIGHT_WHILE_WALKING:
        return false;  // The B1 only changes its height while standing (FORCE_STAND).
    }
    return false;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> RobotDriverUnitreeB1::get_base_orientation_limits() const
{
    return {kBaseOrientationLowerLimits, kBaseOrientationUpperLimits};
}

std::tuple<double, double, double> RobotDriverUnitreeB1::get_base_height_limits() const
{
    return {kNominalBaseHeight - kMaxBaseHeightDelta, kNominalBaseHeight + kMaxBaseHeightDelta, kNominalBaseHeight};
}

std::vector<LeggedRobotDriver::LimbEntry> RobotDriverUnitreeB1::get_limbs() const
{
    return impl_->limbs_;
}

std::vector<bool> RobotDriverUnitreeB1::get_commandable_limbs() const
{
    return std::vector<bool>(impl_->limbs_.size(), false);
}

}
