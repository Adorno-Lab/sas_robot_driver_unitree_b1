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
#
# ################################################################
*/

#pragma once
#include <dqrobotics/DQ.h>
#include <memory>




using namespace DQ_robotics;
using namespace Eigen;

class DriverUnitreeB1
{
protected:
    std::atomic_bool* st_break_loops_;
private:
    enum class STATUS{
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };
    STATUS current_status_{STATUS::IDLE};
    std::string status_msg_;

    enum class HIGH_LEVEL_MODE{
        IDLE_DEFAULT_STAND, // 0. idle, default stand
        FORCE_STAND,        // 1. force stand (controlled by dBodyHeight + ypr)
        TARGET_VELOCITY_WALKING, // 2. target velocity walking (controlled by velocity + yawSpeed)
        PATH_MODE_WALKING,     // 4. path mode walking (reserve for future release)
        POSITION_STAND_DOWN,   // 5. position stand down.
        POSITION_STAND_UP,    // 6. position stand up
        DAMPING_MODE,         // 7. damping mode
        RECOVERY_STAND        // 9. recovery stand
    };
    const std::unordered_map<HIGH_LEVEL_MODE, uint8_t> high_level_mode_map_ =
        {
        {HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND,      0},
        {HIGH_LEVEL_MODE::FORCE_STAND,             1},
        {HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, 2},
        {HIGH_LEVEL_MODE::PATH_MODE_WALKING,       4},
        {HIGH_LEVEL_MODE::POSITION_STAND_DOWN,     5},
        {HIGH_LEVEL_MODE::POSITION_STAND_UP,       6},
        {HIGH_LEVEL_MODE::DAMPING_MODE,            7},
        {HIGH_LEVEL_MODE::RECOVERY_STAND,          9},
        };
    const std::unordered_map<uint8_t, HIGH_LEVEL_MODE> high_level_mode_map_inv_ =
        {
        {0, HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND     },
        {1, HIGH_LEVEL_MODE::FORCE_STAND            },
        {2, HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING},
        {4, HIGH_LEVEL_MODE::PATH_MODE_WALKING      },
        {5, HIGH_LEVEL_MODE::POSITION_STAND_DOWN    },
        {6, HIGH_LEVEL_MODE::POSITION_STAND_UP      },
        {7, HIGH_LEVEL_MODE::DAMPING_MODE           },
        {9, HIGH_LEVEL_MODE::RECOVERY_STAND         },
        };

    HIGH_LEVEL_MODE current_high_level_mode_;
    void _command_in_high_level_mode(const HIGH_LEVEL_MODE& high_level_mode,
                                     const double& forward_vel,
                                     const double& side_vel,
                                     const double& yaw_speed);



//void _set_high_level_mode(const HIGH_LEVEL_MODE& high_level_mode);

public:
    enum class MODE{
        None,
        PositionControl,
        VelocityControl,
        ForceControl,
    };

    enum class LEVEL{HIGH, LOW};
    LEVEL level_;
    enum class BRANCH{FR, FL, RR, RL};


public:
    enum class CUSTOM_FLAGS
    {
        FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO,
    };
    std::vector<CUSTOM_FLAGS> custom_flags_;

private:

    class Impl;
    std::shared_ptr<Impl> impl_;




    double high_level_forward_speed_{0};
    double high_level_side_speed_{0};
    double high_level_yaw_speed_{0};

    std::string ip_ {"0.0.0"};
    int port_{0};

    void _show_status();
    bool verbosity_;
    int timeout_in_milliseconds_;
    bool LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_;

    MODE mode_{MODE::None};

    float dt_{0.002};
    unsigned long long motiontime_{0};
    uint32_t tick_{0}; //real-time from motion controller

    int state_of_charge_{0}; // Battery status (0-100%)

    bool communication_established_{false};
    std::vector<unsigned long long> upd_status_{0,0,0,0,0,0,0};


    //std::atomic<bool> finish_control_loop_{false};
    //std::atomic<bool> finish_echo_robot_state_{false};

    std::atomic<bool> finish_motion_to_deinitialize_{false};
    std::atomic<bool> the_robot_is_ready_to_deinitialize_{false};

    DQ IMU_orientation_{1};
    DQ IMU_gyroscope_{0};
    DQ IMU_accelerometer_{0};

    DQ odometry_position_{0};
    double body_height_{0};

    DQ high_level_linear_velocity_{0};
    DQ high_level_angular_velocity_{0};


    void _update_data_from_robot_state();

    template<typename T>
    void _update_joint_data(const T& state);

    template<typename T>
    void _update_IMU_data(const T& state);

    template<typename T>
    void _update_battery_data(const T& state);


    //-------------------------------------------------------------
    //---------------Robot state attributes------------------------
    //----joint positions--(unit: radian)
    VectorXd qFR_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd qFL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRR_ = (VectorXd(3) << 0,0,0).finished();

    //----joint velocities--(unit: radian/second)
    VectorXd qFR_dot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qFL_dot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRL_dot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRR_dot_  = (VectorXd(3) << 0,0,0).finished();

    //----joint accelerations-- (unit: radian/second^2)
    VectorXd qFR_ddot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qFL_ddot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRL_ddot_  = (VectorXd(3) << 0,0,0).finished();
    VectorXd qRR_ddot_  = (VectorXd(3) << 0,0,0).finished();

    //----estimated output joint torques (unit: N.m)
    VectorXd tauFR_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd tauFL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd tauRL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd tauRR_ = (VectorXd(3) << 0,0,0).finished();

    //----motor temperatures
    VectorXd temperatureFR_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd temperatureFL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd temperatureRL_ = (VectorXd(3) << 0,0,0).finished();
    VectorXd temperatureRR_ = (VectorXd(3) << 0,0,0).finished();
    //--------------------------------------------------------------
    //--------------------------------------------------------------




    void _robot_control();
    void _robot_update();
    void _UDPRecv();
    void _UDPSend();

    void _update_udp_status();

    void _set_driver_mode(const MODE& mode, const LEVEL& level);
    void _initialize_high_cmd_variable();


    static bool are_approximately_equal(const double& a, const double& b, const double& epsilon = 1e-6) ;
    bool status_velocities_{false};



public:
    DriverUnitreeB1() = delete;
    DriverUnitreeB1(const DriverUnitreeB1&) = delete;
    DriverUnitreeB1& operator= (const DriverUnitreeB1&) = delete;
    DriverUnitreeB1(std::atomic_bool* st_break_loops,
                    const MODE& mode = MODE::None,
                    const LEVEL& level = LEVEL::HIGH,
                    const bool& verbosity = true,
                    const int& TIMEOUT_IN_MILLISECONDS = 2000,
                    const bool& LIE_DOWN_ROBOT_WHEN_DEINITIALIZE = true,
                    const std::string &TARGET_IP = "192.168.123.220", // For low-level use "192.168.123.10",
                    const int& TARGET_PORT = 8082,              //For low-level use 8007
                    const int& LOCAL_PORT = 8090,
                    const std::vector<CUSTOM_FLAGS>& custom_flags = std::vector<CUSTOM_FLAGS>{});

    std::string get_target_ip() const;
    int get_target_port() const;
    int get_motiontime() const;
    uint32_t get_realtime_controller() const;
    int get_state_of_charge() const;
    std::string get_status_message() const;

    std::vector<unsigned long long> get_udp_status();
    bool get_connection_status();

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> get_leg_joint_positions() const;

    VectorXd get_joint_positions(const BRANCH& branch) const;
    VectorXd get_joint_velocities(const BRANCH& branch) const;
    VectorXd get_joint_accelerations(const BRANCH& branch) const;
    VectorXd get_joint_estimated_torques(const BRANCH& branch) const;
    VectorXd get_joint_temperatures(const BRANCH& branch) const;

    DQ get_IMU_orientation() const;
    DQ get_IMU_gyroscope() const;
    DQ get_IMU_accelerometer() const;
    DQ get_IMU_pose() const;
    VectorXd get_mobile_platform_configuration_from_IMU_pose() const;
    DQ get_high_level_angular_velocity() const;
    DQ get_high_level_linear_velocity() const;

    DQ get_odometry_position() const;
    double get_body_height() const;



    void set_high_level_forward_speed(const double& forward_speed = 0);
    void set_high_level_yaw_speed(const double& yaw_speed = 0);
    void set_high_level_forward_and_yaw_speed(const double& forward_speed = 0,
                                              const double& yaw_speed = 0);

    void set_high_level_speed(const double& forward_speed = 0,
                              const double& side_speed = 0,
                              const double& yaw_speed = 0);

    double get_high_level_forward_speed_reference() const;
    double get_high_level_yaw_speed_reference() const;

    void show_high_mode() const;
    unsigned long long get_motion_time() const;




};


