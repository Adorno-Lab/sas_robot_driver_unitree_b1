
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
//#include <sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "unitree_b1_node.hpp"

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

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_b1");

    try
    {
        sas::RobotDriverUnitreeB1Configuration robot_driver_unitree_b1_configuration;
        sas::get_ros_parameter(node,"mode", robot_driver_unitree_b1_configuration.mode);
        sas::get_ros_parameter(node,"LIE_DOWN_ROBOT_WHEN_DEINITIALIZE", robot_driver_unitree_b1_configuration.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE);
        sas::get_ros_parameter(node,"robot_name", robot_driver_unitree_b1_configuration.robot_name);

        auto robot_driver = std::make_shared<sas::RobotDriverUnitreeB1>(node,
                                                                        robot_driver_unitree_b1_configuration,
                                                                        &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        robot_driver->control_loop();





    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

   // sas::display_signal_handler_none_bug_info(node);
    return 0;


}

