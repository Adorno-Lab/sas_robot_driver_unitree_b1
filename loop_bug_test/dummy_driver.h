#pragma once
#include <atomic>
#include <memory>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

class dummy_driver
{
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
    unsigned long long motiontime_{0};
    unsigned long long frozen_time_{0};
    std::atomic<bool> finish_motion_to_deinitialize_{false};
    std::atomic<bool> the_robot_is_ready_to_deinitialize_{false};
protected:
    std::atomic_bool* st_break_loops_;
    double dt_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_control_;
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_echo_state_;

    void _robot_control();
    void _robot_update();

public:
    dummy_driver() = delete;
    dummy_driver(const dummy_driver&) = delete;
    dummy_driver& operator= (const dummy_driver&) = delete;
    dummy_driver(std::atomic_bool* st_break_loops, const double& time_step);

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    unsigned long long get_motion_time() const;
};


