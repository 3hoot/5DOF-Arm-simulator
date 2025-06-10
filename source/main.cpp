#include "robot_arm/core/robot.hpp"
#include "robot_arm/core/config.hpp"
#include "robot_arm/sim/simulator.hpp"

int main()
{
    using namespace robot_arm;
    core::Robot robot(core::config::ROBOT_JOINTS);
    sim::Simulator simulator(robot);

    simulator.run();

    return 0;
}
