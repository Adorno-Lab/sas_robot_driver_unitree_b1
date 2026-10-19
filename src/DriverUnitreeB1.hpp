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
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>




using namespace DQ_robotics;
using namespace Eigen;

/**
 * @brief Driver for the Unitree B1 quadruped, built on the Unitree legged SDK (UDP communication).
 *
 * @details Threading model. Each thread runs one complete UDP cycle per tick (receive, update the
 *          robot state, command, send), so the send and receive steps never run concurrently:
 *          - state_thread_ (a ThreadManager) receives the robot state and sends the current command
 *            without changing it. It runs from connect() until a control mode takes over in
 *            initialize() (or for the whole session in MODE::None).
 *          - control_thread_ (a ThreadManager) also commands the robot according to the selected MODE.
 *            It runs from initialize() until deinitialize().
 *          The robot state and the high-level speed references are guarded by data_mutex_, so the
 *          getters and setters are safe to call from any thread.
 */
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

    HIGH_LEVEL_MODE current_high_level_mode_{HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND};
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
    unsigned long long quick_stop_duration_ = 1500; //  smaller = faster to deinitialize, but less controlled stop
    const unsigned long long lie_down_phase_duration_ = 3000; // Duration of each phase of the lie-down sequence (ms)
    const unsigned long long deinitialize_timeout_margin_ = 3000; // Extra time (ms) deinitialize() waits for the stop sequence

    // Ranges of the HighCmd velocities (see unitree_legged_sdk/comm.h)
    const double max_forward_speed_ = 1.2;
    const double min_forward_speed_ = -0.8;
    const double max_side_speed_ = 0.25;
    const double max_yaw_speed_ = 0.75;

    MODE mode_{MODE::None};

    float dt_{0.002};
    // Elapsed time (ms) added to motiontime_ in each tick of the threads
    const unsigned long long motiontime_increment_ = 2;
    const marinholab::sas::core::ThreadManager::PRIORITY thread_priority_ =
        marinholab::sas::core::ThreadManager::PRIORITY::REALTIME;
    std::atomic<unsigned long long> motiontime_{0};
    uint32_t tick_{0}; //real-time from motion controller

    int state_of_charge_{0}; // Battery status (0-100%)

    std::atomic<bool> communication_established_{false};
    std::vector<unsigned long long> upd_status_{0,0,0,0,0,0,0};
    // upd_status_ is written by both the UDP send and the UDP receive threads.
    mutable std::mutex udp_status_mutex_;

    // Protects the robot state and the high-level speed references, which are written by the
    // SDK loop threads and read by the user thread (and vice versa).
    mutable std::mutex data_mutex_;


    //std::atomic<bool> finish_control_loop_{false};
    //std::atomic<bool> finish_echo_robot_state_{false};

    std::atomic<bool> finish_motion_to_deinitialize_{false};
    std::atomic<bool> the_robot_is_ready_to_deinitialize_{false};

    // Used only by the control loop thread to time the stop sequence of deinitialize().
    bool deinitialize_sequence_started_{false};
    unsigned long long deinitialize_start_time_{0};
    int deinitialize_phase_{-1};

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
    double speed_threshold_to_force_stand_mode_ = 0.1;



    void _robot_control();
    void _robot_update();

    //-------To handle the threads-----------------
    std::unique_ptr<marinholab::sas::core::ThreadManager> state_thread_;
    void _start_state_thread();
    void _stop_state_thread();
    void _state_loop_callback();

    std::unique_ptr<marinholab::sas::core::ThreadManager> control_thread_;
    void _start_control_thread();
    void _stop_control_thread();
    void _control_loop_callback();

    // Used only by the threads, to report the first exception caught in their callbacks.
    std::atomic<bool> loop_exception_reported_{false};
    void _report_loop_exception(const std::string& thread_name, const std::string& what);

    void _update_udp_status();

    void _set_driver_mode(const MODE& mode, const LEVEL& level);
    void _initialize_high_cmd_variable();


    static bool are_approximately_equal(const double& a, const double& b, const double& epsilon = 1e-6) ;
    bool status_velocities_{false};



public:
    DriverUnitreeB1() = delete;
    ~DriverUnitreeB1();
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


