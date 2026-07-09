#pragma once
#include <sas_core/sas_robot_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/DQ.h>

using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{


class FreeFlyingRobotDriver: public RobotDriver
{

public:
    /**
     * @brief Enumeration of optional driver functionalities in high level mode
     */
    enum class Mode{
        Idle=0,
        Stance,
        Walking
    };
public:
    FreeFlyingRobotDriver(const FreeFlyingRobotDriver&)=delete;
    FreeFlyingRobotDriver()=delete;
    ~FreeFlyingRobotDriver();

    FreeFlyingRobotDriver(std::shared_ptr<Node>& node,
                          std::atomic_bool* break_loops);

    void set_twist(const DQ& twist);
    DQ get_twist();


};
}
