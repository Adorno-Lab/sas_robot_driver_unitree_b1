#include "sas_robot_driver_unitree_b1/FreeFlyingRobotDriver.hpp"

namespace sas
{

FreeFlyingRobotDriver::~FreeFlyingRobotDriver()
{

}

FreeFlyingRobotDriver::FreeFlyingRobotDriver(
    std::atomic_bool* break_loops)
:RobotDriver{break_loops}
{


}

FreeFlyingRobotDriver::FreeFlyingRobotDriver(const std::shared_ptr<ShutdownSignaler> &shutdown_signaler)
    :RobotDriver{shutdown_signaler}
{

}


}
