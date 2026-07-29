#include "dummy_driver.h"
#include <iostream>



dummy_driver::dummy_driver(std::atomic_bool *st_break_loops, const double &time_step)
    :st_break_loops_{st_break_loops},dt_{time_step}
{
    loop_control_    = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("control_loop",
                                                                   dt_,
                                                                   boost::bind(&dummy_driver::_robot_control,this));

    loop_echo_state_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("echo_state_loop",
                                                                      dt_, boost::bind(&dummy_driver::_robot_update,this));
}

void dummy_driver::connect()
{
    loop_echo_state_->start();
    std::cout<<"Connected!"<<std::endl;
}

void dummy_driver::initialize()
{
    loop_echo_state_->shutdown();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    loop_control_->start();
}

void dummy_driver::deinitialize()
{
    finish_motion_to_deinitialize_ = true;
    while(!the_robot_is_ready_to_deinitialize_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto start = std::chrono::steady_clock::now();
    loop_echo_state_->shutdown();
    auto end = std::chrono::steady_clock::now();
    std::cout << "shutdown() took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;
}

void dummy_driver::disconnect()
{
    std::cerr<<"disconnected!"<<std::endl;
}

unsigned long long dummy_driver::get_motion_time() const
{
    return motiontime_;
}



void dummy_driver::_robot_control()
{
    motiontime_++;
    if (!finish_motion_to_deinitialize_)
    {

    }else{
        // This part of the code is executed when the driver is deinitialized.
        frozen_time_ = motiontime_;
        the_robot_is_ready_to_deinitialize_ = true;
    }

}

void dummy_driver::_robot_update()
{
    motiontime_++;
    //if (finish_motion_to_deinitialize_)
    //    the_robot_is_ready_to_deinitialize_ = true;

    inside_callback = true;

    std::cout << "Entered callback\n";

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "Leaving callback\n";

    inside_callback = false;
}
