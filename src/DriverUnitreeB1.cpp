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
# ################################################################*/

#include "DriverUnitreeB1.hpp"
#include <unitree_legged_sdk/unitree_legged_sdk.h>


class DriverUnitreeB1::Impl
{
public:
    std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safe_;
    std::shared_ptr<UNITREE_LEGGED_SDK::UDP>    udp_;
    UNITREE_LEGGED_SDK::LowCmd low_cmd_;
    UNITREE_LEGGED_SDK::LowState low_state_;

    UNITREE_LEGGED_SDK::HighCmd high_cmd_;
    UNITREE_LEGGED_SDK::HighState high_state_;

    std::vector<int> FR_index_ = {UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2};
    std::vector<int> FL_index_ = {UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2};
    std::vector<int> RL_index_ = {UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2};
    std::vector<int> RR_index_ = {UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2};

    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_control_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_echo_state_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpSend_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_udpRecv_;
    Impl()
    {

    };
};


/**
 * @brief DriverUnitreeB1::DriverUnitreeB1 constructor of the class
 * @param st_break_loops Use this flag to break internal loops. This is useful to stop the robot using a signal
 *              interruption by the user.
 * @param mode  The operation mode. Select the control strategy to command the robot.
 * @param level The control level to be used. HIGH or LOW.
 *              The HIGH level is used to send task space commands
 *              to the robot. The robot constraints are handled by the own robot  using a Unitree internal controller.
 *              The LOW mode is used to send joint position, velocity or torque commands. In this case, you must take into account
 *              all constraints in your controler (joint limits, control input limits, robot balance, self-collision avoidance, etc).
 * @param verbosity Use true (default) to display more information in the terminal.
 * @param TIMEOUT_IN_MILLISECONDS The max time in milliseconds to establish the communication to the robot before to throw an exception.
 * @param LIE_DOWN_ROBOT_WHEN_DEINITIALIZE Use this flag to put the robot on the ground when the driver is deinitialized.
 * @param TARGET_IP The IP address of the B1 robot to perform the communication. You can use a
 *                  LAN cable connection or a WiFI network to establish the communication.
 *
 * @param TARGET_PORT The default port of the B1 robot.
 * @param LOCAL_PORT The communication port of the PC that is running the code.
 *
 *
 *
 *
 * Example:
 *
 *          // This class follows the SmartArmStack driver principles, in which four methods are required to
 *          // start and finish the robot communication.
 *
            DriverUnitreeB1 B1(&kill_this_process,
                            DriverUnitreeB1::MODE::VelocityControl, // Driver mode
                            DriverUnitreeB1::LEVEL::HIGH,       // Level mode
                            true,   //verbosity
                            2000,   // TIMEOUT in ms
                            false, // LIE DOWN ROBOT WHEN DEINITIALIZE
                            "192.168.123.220",  // Target IP   //192.168.123.10 for low-level mode
                            8082,              // Target port  //8007 for low-level mode
                            8090);             // Local port


            B1.connect();     // First method to be called.
            B1.initialize();  // Second method to be called. It is required to connect before to initialize.

            //Your code here
            B1.set_high_level_speed(-0.03, 0, 0.0);  //0.03 is the minimum value in forward speed.
            //

            B1.deinitialize();  // Third method to be called.
            B1.disconnect();    // Fourth method to be called. It is required to deinitialize before to disconnect.

 */
DriverUnitreeB1::DriverUnitreeB1(std::atomic_bool *st_break_loops,
                                           const MODE &mode,
                                           const LEVEL &level,
                                           const bool &verbosity,
                                           const int &TIMEOUT_IN_MILLISECONDS,
                                           const bool &LIE_DOWN_ROBOT_WHEN_DEINITIALIZE,
                                           const string &TARGET_IP,
                                           const int &TARGET_PORT,
                                           const int &LOCAL_PORT):
    st_break_loops_{st_break_loops},
    ip_{TARGET_IP},
    port_{TARGET_PORT},
    verbosity_{verbosity},
    timeout_in_milliseconds_{TIMEOUT_IN_MILLISECONDS},
    LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_{LIE_DOWN_ROBOT_WHEN_DEINITIALIZE}
{
    impl_        = std::make_shared<DriverUnitreeB1::Impl>();
    impl_->safe_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::B1);
    impl_->udp_  = std::make_shared<UNITREE_LEGGED_SDK::UDP>
        (level == LEVEL::LOW ? UNITREE_LEGGED_SDK::LOWLEVEL : UNITREE_LEGGED_SDK::HIGHLEVEL,LOCAL_PORT,TARGET_IP.c_str(), TARGET_PORT);


    current_status_ = STATUS::IDLE;
    status_msg_ = std::string("Idle.");
    _set_driver_mode(mode, level);

    if (level == LEVEL::LOW)
        impl_->udp_->InitCmdData(impl_->low_cmd_);
    else
        impl_->udp_->InitCmdData(impl_->high_cmd_);

    UNITREE_LEGGED_SDK::InitEnvironment();

    /*
    UNITREE_LEGGED_SDK::LoopFunc loop_control("control_loop", dt_, boost::bind(&RobotDriverUnitreeB1::RobotControl, this));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpSend("udp_send", dt_, 3,  boost::bind(&RobotDriverUnitreeB1::UDPSend, this));
    UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv("udp_recv", dt_, 3,  boost::bind(&RobotDriverUnitreeB1::UDPRecv, this));
    */
    impl_->loop_control_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("control_loop", dt_, boost::bind(&DriverUnitreeB1::_robot_control,this));
    impl_->loop_echo_state_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("echo_state_loop", dt_, boost::bind(&DriverUnitreeB1::_robot_update,this));
    impl_->loop_udpSend_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("udp_send", dt_, 3,  boost::bind(&DriverUnitreeB1::_UDPSend, this));
    impl_->loop_udpRecv_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("udp_recv", dt_, 3,  boost::bind(&DriverUnitreeB1::_UDPRecv, this));

    ip_ = TARGET_IP;
    port_ = TARGET_PORT;


}

/**
 * @brief DriverUnitreeB1::get_target_ip returns the Robot IP (TARGET_IP) address. For high-level mode,
 *                  the IP address is usually "192.168.123.220" and "192.168.123.10" for low-level mode.
 * @return The robot IP address.
 */
std::string DriverUnitreeB1::get_target_ip() const
{
    return ip_;
}


/**
 * @brief DriverUnitreeB1::get_target_port returns the robot port (target port). Usually this value
 *                  corresponds to 8082 for high-level mode and 8007 for low-level driver.
 * @return
 */
int DriverUnitreeB1::get_target_port() const
{
    return port_;
}


/**
 * @brief DriverUnitreeB1::get_motiontime returns the elapsed time in the control loop thread.
 * @return
 */
int DriverUnitreeB1::get_motiontime() const
{
    return motiontime_;
}

/**
 * @brief DriverUnitreeB1::get_realtime_controller returns the elapsed time in the low-level controller.
 * @return
 */
uint32_t DriverUnitreeB1::get_realtime_controller() const
{
    return tick_;
}

/**
 * @brief DriverUnitreeB1::get_state_of_charge returns the state of charge of the B1 battery.
 * @return A value from 0-100%
 */
int DriverUnitreeB1::get_state_of_charge() const
{
    return state_of_charge_;
}

/**
 * @brief DriverUnitreeB1::get_status_message returns the status message of the driver.
 * @return
 */
std::string DriverUnitreeB1::get_status_message() const
{
    return status_msg_;
}


/**
 * @brief DriverUnitreeB1::get_udp_status returns the UDP communication status.
 * @return a 7-dimensional vector containing the UDP communication status. The vector containts
 *      the following ordered data:
 *
 *       unsigned long long TotalCount;	  // total loop count
 *       unsigned long long SendCount;	  // total send count
 *       unsigned long long RecvCount;	  // total receive count
 *       unsigned long long SendError;	  // total send error
 *       unsigned long long FlagError;	  // total flag error
 *       unsigned long long RecvCRCError;  // total reveive CRC error
 *       unsigned long long RecvLoseError; // total lose package count
 */
std::vector<unsigned long long> DriverUnitreeB1::get_udp_status()
{
    return upd_status_;
}

/**
 * @brief DriverUnitreeB1::get_connection_status returns a flag that represents the connection status.
 * @return The connection status flag. Returns true if the connection was successful. False otherwise.
 */
bool DriverUnitreeB1::get_connection_status()
{
    return communication_established_;
}

/**
 * @brief DriverUnitreeB1::_UDPRecv receives data from the UDP communication. This callback method is used by loop_udpRecv_.
 */
void DriverUnitreeB1::_UDPRecv()
{
    impl_->udp_->Recv();
    _update_udp_status();
}

/**
 * @brief DriverUnitreeB1::_UDPSend sends data using the UDP communication. This callback method is used by loop_udpSend_.
 */
void DriverUnitreeB1::_UDPSend()
{
    impl_->udp_->Send();
    _update_udp_status();
}

/**
 * @brief DriverUnitreeB1::_update_udp_status updates the UDP status if the data it is initialized.
 *
 */
void DriverUnitreeB1::_update_udp_status()
{
    upd_status_.at(0) = impl_->udp_->udpState.TotalCount >0 ? impl_->udp_->udpState.TotalCount : 0;
    upd_status_.at(1) = impl_->udp_->udpState.SendCount  >0 ? impl_->udp_->udpState.SendCount : 0;
    upd_status_.at(2) = impl_->udp_->udpState.RecvCount  >0 ? impl_->udp_->udpState.RecvCount : 0;
    upd_status_.at(3) = impl_->udp_->udpState.SendError  >0 ? impl_->udp_->udpState.SendError : 0;
    upd_status_.at(4) = impl_->udp_->udpState.FlagError  >0 ? impl_->udp_->udpState.FlagError : 0;
    upd_status_.at(5) = impl_->udp_->udpState.RecvCRCError >0 ? impl_->udp_->udpState.RecvCRCError : 0;
    upd_status_.at(6) = impl_->udp_->udpState.RecvLoseError >0 ? impl_->udp_->udpState.RecvLoseError : 0;
}


/**
 * @brief DriverUnitreeB1::_set_driver_mode sets the driver mode.
 * @param mode The operation mode. Select the control strategy to command the robot.
 * @param level The control level to be used. HIGH or LOW.
 *              The HIGH level is used to send task space commands
 *              to the robot. The robot constraints are handled by the own robot  using a Unitree internal controller.
 *              The LOW mode is used to send joint position, velocity or torque commands. In this case, you must take into account
 *              all constraints in your controler (joint limits, control input limits, robot balance, self-collision avoidance, etc).
 */
void DriverUnitreeB1::_set_driver_mode(const MODE &mode, const LEVEL &level)
{
    switch (mode)
    {
        case MODE::None:
            std::cerr<<"RobotDriverUnitreeB1::_set_driver_mode. Driver is set to Mode::None. "<<std::endl;
            break;
        case MODE::PositionControl:
            if (level == LEVEL::LOW)
            {
                throw std::runtime_error(std::string("RobotDriverUnitreeB1::_set_driver_mode. PositionControl in low-level mode is not available. "));
            }else { //HIGH LEVEL
                throw std::runtime_error(std::string("RobotDriverUnitreeB1::_set_driver_mode. PositionControl in high-level mode is not available. "));
            }

            break;
        case MODE::VelocityControl:
            if (level == LEVEL::LOW)
            {
                throw std::runtime_error(std::string("RobotDriverUnitreeB1::_set_driver_mode. VelocityControl in low-level mode is not available. "));
            }else { //HIGH LEVEL
                std::cerr<<"RobotDriverUnitreeB1::_set_driver_mode. VelocityControl in high-level mode is experimental. "<<std::endl;
                _initialize_high_cmd_variable();
            }
            break;
        case MODE::ForceControl:
            if (level == LEVEL::LOW)
            {
                throw std::runtime_error(std::string("RobotDriverUnitreeB1::_set_driver_mode. ForceControl in low-level mode is not available. "));
            }else { //HIGH LEVEL
                throw std::runtime_error(std::string("RobotDriverUnitreeB1::_set_driver_mode. ForceControl in high-level mode is not available. "));
            }

            break;
    }
    mode_ = mode;
    level_ = level;
}


/**
 * @brief DriverUnitreeB1::_initialize_high_cmd_variable sets the high_cmd_ (UNITREE_LEGGED_SDK::HighCmd struct) attribute with zeros.
 */
void DriverUnitreeB1::_initialize_high_cmd_variable()
{
    impl_->high_cmd_.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    impl_->high_cmd_.gaitType = 0;
    impl_->high_cmd_.speedLevel = 0;
    impl_->high_cmd_.footRaiseHeight = 0;
    impl_->high_cmd_.bodyHeight = 0;
    impl_->high_cmd_.euler[0] = 0;
    impl_->high_cmd_.euler[1] = 0;
    impl_->high_cmd_.euler[2] = 0;
    impl_->high_cmd_.velocity[0] = 0.0f;
    impl_->high_cmd_.velocity[1] = 0.0f;
    impl_->high_cmd_.yawSpeed = 0.0f;
    impl_->high_cmd_.reserve = 0;
}

/**
 * @brief DriverUnitreeB1::connect This method starts the threads related to the UPD communication between the PC running the
 *                          controller and the B1 hardware (motors, sensors, battery, etc). Furthermore, a thread to update the robot state
 *                          is started automatically. At this stage, there are no control loops running, and consequently, the robot
 *                          will not perform any movement.
 */
void DriverUnitreeB1::connect()
{
    if (current_status_ == STATUS::IDLE)
    {
        impl_->loop_udpSend_->start();
        impl_->loop_udpRecv_->start();
        impl_->loop_echo_state_->start();

        status_msg_ = "connecting...";
        for (int i=0;i<timeout_in_milliseconds_;i++)
        {
            if (get_udp_status().at(2) > 0)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        _show_status();
        if (get_udp_status().at(2) > 0)
        {
            current_status_ = STATUS::CONNECTED;
            status_msg_ = "connected!";
            communication_established_ = true;
            _show_status();

        }else
        {
            current_status_ = STATUS::IDLE;
            status_msg_ = "Connection failed!";
            impl_->loop_udpSend_->shutdown();
            impl_->loop_udpRecv_->shutdown();
            impl_->loop_echo_state_->shutdown();
            _show_status();
            //throw std::runtime_error("Unestablished connection with the B1 robot!");
        }


    }
}

/**
 * @brief DriverUnitreeB1::initialize starts the appropriate threads based on the selected mode of operation.
 *                  This method requires established communication with the robot, i.e. the user must call connect()
 *                  before calling initialize().
 *                  If the operation mode is different from None, the robot may move!
 *                  WARNING: Be prepared to stop the robot with an emergency stop protocol!
 */
void DriverUnitreeB1::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        switch (mode_)
        {
            case MODE::None:
                break;
            case MODE::PositionControl:
                if (level_ == LEVEL::LOW)
                {
                    std::cerr<<"RobotDriverUnitreeB1::initialize. PositionControl  in low-level mode is not available. "<<std::endl;
                    deinitialize();
                }else{ //HIGH LEVEL
                    std::cerr<<"RobotDriverUnitreeB1::initialize. PositionControl  in high-level mode is not available. "<<std::endl;
                    deinitialize();
                    }
                break;
            case MODE::VelocityControl:

                if (level_ == LEVEL::LOW)
                {
                    std::cerr<<"RobotDriverUnitreeB1::initialize. VelocityControl in low-level mode is not available. "<<std::endl;
                    deinitialize();
                }else { //HIGH LEVEL
                        status_msg_ = "finishing echo state loop.";
                        _show_status();
                        impl_->loop_echo_state_->shutdown();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        impl_->loop_control_->start();
                        status_msg_ = "starting control loop.";
                        _show_status();
                    }
                break;
            case MODE::ForceControl:
                if (level_ == LEVEL::LOW)
                {
                    std::cerr<<"RobotDriverUnitreeB1::initialize. ForceControl in low-level mode is not available. "<<std::endl;
                    deinitialize();
                }else{
                    std::cerr<<"RobotDriverUnitreeB1::initialize. ForceControl in high-level mode is not available. "<<std::endl;
                    deinitialize();
                }
                break;
        }
        current_status_ = STATUS::INITIALIZED;
        status_msg_ = "Initialized!";
    }else{
        std::cerr<<"RobotDriverUnitreeB1::initialize. The driver must be connected before to be initialized. "<<std::endl;
    }
}


/**
 * @brief DriverUnitreeB1::deinitialize stops all communication threads.
 *                  This method requires initialized communication with the robot, i.e. the user must call both connect(),
 *                  and initialize() before calling deinitialize().
 */
void DriverUnitreeB1::deinitialize()
{
    //wait to finish;
    finish_motion_to_deinitialize_ = true;
    std::cerr<<"Waiting to deinitialize..."<<std::endl;
    while(!the_robot_is_ready_to_deinitialize_){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

    std::cerr<<"We are ready to deinitialize!"<<std::endl;
    impl_->loop_udpSend_->shutdown();
    impl_->loop_udpRecv_->shutdown();
    impl_->loop_echo_state_->shutdown();
    impl_->loop_control_->shutdown();

    status_msg_ = "All loops are shutdown!";
    _show_status();

    current_status_ = STATUS::DEINITIALIZED;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    status_msg_ = "Deinitialized!";
    _show_status();
}

/**
 * @brief DriverUnitreeB1::disconnect
 */
void DriverUnitreeB1::disconnect()
{
    current_status_ = STATUS::DISCONNECTED;
    status_msg_ = "Disconnected!";
    _show_status();
}

/**
 * @brief DriverUnitreeB1::get_leg_joint_positions returns the joint positions of the robot legs.
 * @return A tuple containing the robot configuration legs in the following order: Front Right, Front Left, Rear Right, Rear Left.
 */
std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> DriverUnitreeB1::get_leg_joint_positions() const
{
    //uFR, uFL, uRR, uRL
    return {qFR_, qFL_, qRR_, qRL_};
}

/**
 * @brief DriverUnitreeB1::get_joint_positions returns the joint positions of the specified leg.
 * @param branch The desired branch (leg). You can use FR (forward right), FL (forward left), RR (rear right), or RL (rear left).
 * @return the current joint positions (unit: radian)
 */
VectorXd DriverUnitreeB1::get_joint_positions(const BRANCH &branch) const
{
    switch (branch){

    case BRANCH::FR:
        return qFR_;
    case BRANCH::FL:
        return qFL_;
    case BRANCH::RR:
        return qRR_;
    case BRANCH::RL:
        return qRL_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeB1::get_joint_positions");
        break;
    }
}

/**
 * @brief DriverUnitreeB1::get_joint_velocities returns the joint velocities of the specified leg.
 * @param branch The desired branch (leg). You can use FR (forward right), FL (forward left), RR (rear right), or RL (rear left).
 * @return the current joint velocities (unit: radian/second)
 */
VectorXd DriverUnitreeB1::get_joint_velocities(const BRANCH &branch) const
{
    switch (branch){

    case BRANCH::FR:
        return qFR_dot_;
    case BRANCH::FL:
        return qFL_dot_;
    case BRANCH::RR:
        return qRR_dot_;
    case BRANCH::RL:
        return qRL_dot_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeB1::get_joint_velocities");
        break;
    }
}


/**
 * @brief DriverUnitreeB1::get_joint_accelerations returns the joint accelerations of the specified leg.
 * @param branch The desired branch (leg). You can use FR (forward right), FL (forward left), RR (rear right), or RL (rear left).
 * @return the current joint accelerations (unit: radian/second^2)
 */
VectorXd DriverUnitreeB1::get_joint_accelerations(const BRANCH &branch) const
{
    switch (branch){

    case BRANCH::FR:
        return qFR_ddot_;
    case BRANCH::FL:
        return qFL_ddot_;
    case BRANCH::RR:
        return qRR_ddot_;
    case BRANCH::RL:
        return qRL_ddot_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeB1::get_joint_accelerations");
        break;
    }

}

/**
 * @brief DriverUnitreeB1::get_joint_estimated_torques returns the estimated joint torques of the specified leg.
 * @param branch The desired branch (leg). You can use FR (forward right), FL (forward left), RR (rear right), or RL (rear left).
 * @return the estimated joint torques (unit: N.m)
 */
VectorXd DriverUnitreeB1::get_joint_estimated_torques(const BRANCH &branch) const
{
    switch (branch){

    case BRANCH::FR:
        return tauFR_;
    case BRANCH::FL:
        return tauFL_;
    case BRANCH::RR:
        return tauRR_;
    case BRANCH::RL:
        return tauRL_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeB1::get_joint_estimated_torques");
        break;
    }
}


/**
 * @brief DriverUnitreeB1::get_joint_temperatures returns the motor temperatures of the specified leg.
 * @param branch The desired branch (leg). You can use FR (forward right), FL (forward left), RR (rear right), or RL (rear left).
 * @return The motor temperatures.
 */
VectorXd DriverUnitreeB1::get_joint_temperatures(const BRANCH &branch) const
{
    switch (branch){

    case BRANCH::FR:
        return temperatureFR_;
    case BRANCH::FL:
        return temperatureFL_;
    case BRANCH::RR:
        return temperatureRR_;
    case BRANCH::RL:
        return temperatureRL_;
    default: // This line is required in GNU/Linux
        throw std::runtime_error("Wrong arguments in RobotDriverUnitreeB1::get_joint_estimated_temperatures");
        break;
    }
}

/**
 * @brief DriverUnitreeB1::get_IMU_orientation returns the IMU-based estimated orientation.
 * @return
 */
DQ DriverUnitreeB1::get_IMU_orientation() const
{
    return IMU_orientation_;
}

/**
 * @brief DriverUnitreeB1::get_IMU_gyroscope returns the IMU-based estimated velocities.
 * @return
 */
DQ DriverUnitreeB1::get_IMU_gyroscope() const
{
    return IMU_gyroscope_;
}

/**
 * @brief DriverUnitreeB1::get_IMU_accelerometer returns the IMU-based estimated accelerations.
 * @return
 */
DQ DriverUnitreeB1::get_IMU_accelerometer() const
{
    return IMU_accelerometer_;
}

/**
 * @brief DriverUnitreeB1::get_odometry_position returns the IMU-based estimated robot position.
 * @return
 */
DQ DriverUnitreeB1::get_odometry_position() const
{
    return odometry_position_;
}

/**
 * @brief DriverUnitreeB1::get_body_height returns the estimated body height.
 * @return
 */
double DriverUnitreeB1::get_body_height() const
{
    return body_height_;
}

/**
 * @brief DriverUnitreeB1::get_IMU_pose returns the estimated robot pose.
 * @return
 */
DQ DriverUnitreeB1::get_IMU_pose() const
{
    DQ r = IMU_orientation_;

    DQ p = 0.5*k_; // for debug
    //DQ hoffset = body_height_*k_ + 0*k_;
    if (level_ == LEVEL::HIGH)
    {
        auto auxp = odometry_position_;
        auto vec_auxp = odometry_position_.vec4();
        auto x = vec_auxp(1);
        auto y = vec_auxp(2);
        auto z = body_height_ + 0.025;
        p = x*i_ + y*j_ + z*k_;
    }
    else
        p = 0.5*k_;
    return (r + E_*0.5*p*r).normalize();
}


/**
 * @brief DriverUnitreeB1::get_mobile_platform_configuration_from_IMU_pose returns the configuration of the holonomic mobile platform.
 * @return A vector containing the x-position, y-position, and the rotation (yaw) angle.
 */
VectorXd DriverUnitreeB1::get_mobile_platform_configuration_from_IMU_pose() const
{
    auto x = get_IMU_pose();
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}


/**
 * @brief DriverUnitreeB1::get_high_level_angular_velocity returns the angular velocities when in High level mode
 * @return The angular velocity (yaw_speed*k_)
 */

DQ DriverUnitreeB1::get_high_level_angular_velocity() const
{
    return high_level_angular_velocity_;
}

/**
 * @brief DriverUnitreeB1::get_high_level_linear_velocity returns the linear velocities when in High level mode
 * @return The planar joint velocties (x_dot*i_ + y_dot*j_)
 */
DQ DriverUnitreeB1::get_high_level_linear_velocity() const
{
    return high_level_linear_velocity_;
}


/**
 * @brief DriverUnitreeB1::set_high_level_forward_speed sets the target forward speed of the holonomic mobile platform.
 * @param forward_speed The desired forward speed. This method is used when the driver is set in high-level.
 */
void DriverUnitreeB1::set_high_level_forward_speed(const double &forward_speed)
{
    high_level_forward_speed_ = forward_speed;
}

/**
 * @brief DriverUnitreeB1::set_high_level_yaw_speed sets the yaw speed of the holonomic mobile platform.
 * @param yaw_speed The desired yaw speed. This method is used when the driver is set in high-level.
 */
void DriverUnitreeB1::set_high_level_yaw_speed(const double &yaw_speed)
{
    high_level_yaw_speed_ = yaw_speed;
}

/**
 * @brief DriverUnitreeB1::set_high_level_forward_and_yaw_speed sets the target forward and yaw speeds of the holonomic mobile platform.
 *                         This method is used when the driver is set in high-level.
 * @param forward_speed The desired forward speed.
 * @param yaw_speed The desired yaw speed.
 */
void DriverUnitreeB1::set_high_level_forward_and_yaw_speed(const double &forward_speed, const double &yaw_speed)
{
    set_high_level_forward_speed(forward_speed);
    set_high_level_yaw_speed(yaw_speed);
}

/**
 * @brief DriverUnitreeB1::set_high_level_speed sets the target forward, side and yaw speeds of the holonomic mobile platform.
 *                         This method is used when the driver is set in high-level.
 * @param forward_speed The desired forward speed.
 * @param side_speed  The desired side speed.
 * @param yaw_speed The desired yaw speed.
 */
void DriverUnitreeB1::set_high_level_speed(const double &forward_speed, const double &side_speed, const double &yaw_speed)
{
    high_level_forward_speed_ = forward_speed;
    high_level_side_speed_ = side_speed;
    high_level_yaw_speed_ = yaw_speed;
}

/**
 * @brief DriverUnitreeB1::get_high_level_forward_speed_reference returns the mobile platform velocities.
 * @return
 */
double DriverUnitreeB1::get_high_level_forward_speed_reference() const
{
    return high_level_forward_speed_;
}

/**
 * @brief DriverUnitreeB1::get_high_level_yaw_speed_reference returns the yaw speed of the mobile platform.
 * @return
 */
double DriverUnitreeB1::get_high_level_yaw_speed_reference() const
{
    return high_level_yaw_speed_;
}

/**
 * @brief DriverUnitreeB1::show_high_mode displays the current high-level mode
 */
void DriverUnitreeB1::show_high_mode() const
{
    switch(current_high_level_mode_){

    case HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND:
        std::cerr<<"IDLE_DEFAULT_STAND"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::FORCE_STAND:
        std::cerr<<"FORCE_STAND"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING:
        std::cerr<<"TARGET_VELOCITY_WALKING"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::PATH_MODE_WALKING:
        std::cerr<<"PATH_MODE_WALKING"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::POSITION_STAND_DOWN:
        std::cerr<<"POSITION_STAND_DOWN"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::POSITION_STAND_UP:
        std::cerr<<"POSITION_STAND_UP"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::DAMPING_MODE:
        std::cerr<<"DAMPING_MODE"<<std::endl;
        break;
    case HIGH_LEVEL_MODE::RECOVERY_STAND:
        std::cerr<<"RECOVERY_STAND"<<std::endl;
        break;
    }
}

/**
 * @brief DriverUnitreeB1::get_motion_time returns the elapsed time of the thread control loop.
 * @return
 */
unsigned long long DriverUnitreeB1::get_motion_time() const
{
    return motiontime_;
}




/**
 * @brief DriverUnitreeB1::_robot_control callback method used by the thread control loop.
 */
void DriverUnitreeB1::_robot_control()
{
    motiontime_ += 2;//motiontime_++;
    _update_data_from_robot_state();
    switch (mode_)
    {
    case MODE::None:
        break;
    case MODE::PositionControl:
        break;
    case MODE::VelocityControl:
        if (level_ == LEVEL::LOW)
        {
            throw std::runtime_error(std::string("RobotDriverUnitreeB1::_set_driver_mode. VelocityControl in low-level mode is not available. "));
        }
        else
        { //HIGH LEVEL
            if (!finish_motion_to_deinitialize_)
            {
                _command_in_high_level_mode(HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, high_level_forward_speed_, high_level_side_speed_, high_level_yaw_speed_);

            }else{
                // This part of the code is executed when the driver is deinitialized.
                static unsigned long long frozen_time = motiontime_;
                const int deltatime = 3000;
                if (motiontime_>= frozen_time && motiontime_ < frozen_time+deltatime)
                {
                    //show_high_mode();
                    std::cout<<"Stopping...  "<< frozen_time+deltatime-motiontime_<<std::endl;
                    _command_in_high_level_mode(HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING, 0, 0, 0);//Stop the robot
                }
                else if (motiontime_>= frozen_time+deltatime && motiontime_ < frozen_time+2*deltatime)
                {
                    //show_high_mode();


                    if (current_high_level_mode_ == HIGH_LEVEL_MODE::DAMPING_MODE)
                    {
                        std::cout<<"ROBOT IS DAMPING MODE. I WILL IGNORE THE POSITION_STAND_UP... "<<frozen_time + 2*deltatime -motiontime_<<std::endl;
                    }else
                    {
                        std::cout<<"POSITION_STAND_UP... "<<frozen_time + 2*deltatime -motiontime_<<std::endl;
                        _command_in_high_level_mode(HIGH_LEVEL_MODE::POSITION_STAND_UP, 0, 0, 0); // Stand up pose
                    }
                }
                else if (motiontime_>= frozen_time+2*deltatime && motiontime_ < frozen_time+ 3*deltatime && LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_)
                {
                    //show_high_mode();
                    std::cout<<"POSITION_STAND_DOWN... "<<frozen_time + 3*deltatime -motiontime_<<std::endl;
                    _command_in_high_level_mode(HIGH_LEVEL_MODE::POSITION_STAND_DOWN, 0, 0, 0);//Stand down pose
                }
                else if (motiontime_>= frozen_time+ 3*deltatime && motiontime_ < frozen_time+ 4*deltatime && LIE_DOWN_ROBOT_WHEN_DEINITIALIZE_)
                {
                    //show_high_mode();
                    std::cout<<"DAMPING_MODE... "<<frozen_time + 4*deltatime -motiontime_<<std::endl;
                    _command_in_high_level_mode(HIGH_LEVEL_MODE::DAMPING_MODE, 0, 0, 0);//Damping mode
                }
                else{
                    show_high_mode();
                    std::cout<<"IDLE"<<std::endl;
                    _command_in_high_level_mode(HIGH_LEVEL_MODE::IDLE_DEFAULT_STAND, 0, 0, 0); //IDLE
                    the_robot_is_ready_to_deinitialize_ = true;
                }
            }
        }
        break;
    case MODE::ForceControl:
        break;
    }
}

/**
 * @brief DriverUnitreeB1::_command_in_high_level_mode sets the high_cmd_ struct with the desired target velocities and sents to the robot.
 * @param high_level_mode The high level mode. This can be
 *       IDLE_DEFAULT_STAND,
 *       FORCE_STAND,
 *       TARGET_VELOCITY_WALKING,
 *       PATH_MODE_WALKING,
 *       POSITION_STAND_DOWN,
 *       POSITION_STAND_UP,
 *       DAMPING_MODE,
 *       RECOVERY_STAND
 *
 *
 * @param forward_vel The target forward velocity.
 * @param side_vel The garget side velocity.
 * @param yaw_speed The target yaw velocity.
 */
void DriverUnitreeB1::_command_in_high_level_mode(const HIGH_LEVEL_MODE& high_level_mode,
                                                       const double& forward_vel,
                                                       const double &side_vel,
                                                       const double& yaw_speed)
{
    _initialize_high_cmd_variable();
    impl_->high_cmd_.mode = high_level_mode_map_.at(high_level_mode);
    impl_->high_cmd_.velocity[0] = forward_vel;
    impl_->high_cmd_.velocity[1] = side_vel;
    impl_->high_cmd_.yawSpeed = yaw_speed;
    impl_->udp_->SetSend(impl_->high_cmd_);
}


/**
 * @brief DriverUnitreeB1::_show_status displays the driver status if the verbosity flag is enabled.
 */
void DriverUnitreeB1::_show_status()
{
    if (verbosity_)
        std::cerr<<status_msg_<<std::endl;
}

/**
 * @brief DriverUnitreeB1::_update_data_from_robot_state callback method used in the thread related to the robot state.
 */
void DriverUnitreeB1::_update_data_from_robot_state()
{
    if (level_ == LEVEL::LOW)
    {
        impl_->udp_->GetRecv(impl_->low_state_);
        _update_joint_data(impl_->low_state_);
        _update_battery_data(impl_->low_state_);
        _update_IMU_data(impl_->low_state_);

        // real-time from motion controller
        tick_ = impl_->low_state_.tick;
    }else{
        impl_->udp_->GetRecv(impl_->high_state_);
        _update_joint_data(impl_->high_state_);
        _update_battery_data(impl_->high_state_);
        _update_IMU_data(impl_->high_state_);
        odometry_position_ = impl_->high_state_.position.at(0)*i_+
                             impl_->high_state_.position.at(1)*j_+
                             impl_->high_state_.position.at(2)*k_;
        body_height_ = impl_->high_state_.bodyHeight;
        current_high_level_mode_ = high_level_mode_map_inv_.at(impl_->high_state_.mode);

        high_level_linear_velocity_  = impl_->high_state_.velocity.at(0)*i_+
                                       impl_->high_state_.velocity.at(1)*j_;

        // this value impl_->high_state_.velocity.at(2)*k_ is not working. Therefore, I extracted the angular
        // velocity from yawSpeed.
        high_level_angular_velocity_ = impl_->high_state_.yawSpeed*k_;
    }
}


/**
 * @brief DriverUnitreeB1::_robot_update This method updates the robot state. This includes the
 *        joint positions, velocities, accelerations, torques, and temperatures.
 */
void DriverUnitreeB1::_robot_update()
{
    motiontime_++;
    _update_data_from_robot_state();
    if (finish_motion_to_deinitialize_)
        the_robot_is_ready_to_deinitialize_ = true;
}




/**
 * @brief DriverUnitreeB1::_update_joint_data updates the robot state related to the legs. This includes the joint positions, velocities,
 *                      accelerations, torques, and motor temperatures.
 * @param state The high-level of low-level structure to store the robot data.
 */
template<typename T>
void DriverUnitreeB1::_update_joint_data(const T &state)
{
    for (int i = 0; i<3;i++)
    {
        // Update the joint positions
        qFR_(i) = state.motorState[impl_->FR_index_.at(i)].q;
        qFL_(i) = state.motorState[impl_->FL_index_.at(i)].q;
        qRL_(i) = state.motorState[impl_->RL_index_.at(i)].q;
        qRR_(i) = state.motorState[impl_->RR_index_.at(i)].q;

        // Update the joint velocities
        qFR_dot_(i) = state.motorState[impl_->FR_index_.at(i)].dq;
        qFL_dot_(i) = state.motorState[impl_->FL_index_.at(i)].dq;
        qRL_dot_(i) = state.motorState[impl_->RL_index_.at(i)].dq;
        qRR_dot_(i) = state.motorState[impl_->RR_index_.at(i)].dq;

        // Update the joint accelerations
        qFR_ddot_(i) = state.motorState[impl_->FR_index_.at(i)].ddq;
        qFL_ddot_(i) = state.motorState[impl_->FL_index_.at(i)].ddq;
        qRL_ddot_(i) = state.motorState[impl_->RL_index_.at(i)].ddq;
        qRR_ddot_(i) = state.motorState[impl_->RR_index_.at(i)].ddq;

        // Update the estimated joint torques output
        tauFR_(i) = state.motorState[impl_->FR_index_.at(i)].tauEst;
        tauFL_(i) = state.motorState[impl_->FL_index_.at(i)].tauEst;
        tauRL_(i) = state.motorState[impl_->RL_index_.at(i)].tauEst;
        tauRR_(i) = state.motorState[impl_->RR_index_.at(i)].tauEst;

        temperatureFR_(i) = state.motorState[impl_->FR_index_.at(i)].temperature;
        temperatureFL_(i) = state.motorState[impl_->FL_index_.at(i)].temperature;
        temperatureRL_(i) = state.motorState[impl_->RL_index_.at(i)].temperature;
        temperatureRR_(i) = state.motorState[impl_->RR_index_.at(i)].temperature;
    }
}

/**
 * @brief DriverUnitreeB1::_update_IMU_data updates the IMU-based data
 * @param state The high-level of low-level structure to store the robot data.
 */
template<typename T>
void DriverUnitreeB1::_update_IMU_data(const T &state)
{
    IMU_orientation_ =   DQ(state.imu.quaternion.at(0),
                          state.imu.quaternion.at(1),
                          state.imu.quaternion.at(2),
                          state.imu.quaternion.at(3)).normalize();

    IMU_gyroscope_ =  state.imu.gyroscope.at(0)*i_+
                      state.imu.gyroscope.at(1)*j_+
                      state.imu.gyroscope.at(2)*k_;

    IMU_accelerometer_  = state.imu.accelerometer.at(0)*i_+
                          state.imu.accelerometer.at(1)*j_+
                          state.imu.accelerometer.at(2)*k_;
}

/**
 * @brief DriverUnitreeB1::_update_battery_data updates the battery data.
 * @param state The high-level of low-level structure to store the robot data.
 */
template<typename T>
void DriverUnitreeB1::_update_battery_data(const T &state)
{
    // Update the battery status
    state_of_charge_ = state.bms.SOC;
}

