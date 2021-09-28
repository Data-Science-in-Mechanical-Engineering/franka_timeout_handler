#include "../include/franka_timeout_handler/robot.h"
#include "../include/franka_timeout_handler/constants.h"
#include <iostream>

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./hold IP" << std::endl;
        return 1;
    }

    franka_timeout_handler::Robot robot(argv[1]);
    robot.start(franka_timeout_handler::ControllerType::cartesian);
    robot.move_target_position(franka_timeout_handler::default_target_position + 0.1 * Eigen::Matrix<double, 3, 1>(0.0, 0.0, 1.0), franka_timeout_handler::default_target_orientation, 3000);
    robot.receive();
    size_t call0 = robot.get_call();
    const size_t time = 10000;
    const size_t period = 1000;
    size_t late = 0;
    for (size_t i = 0; i < time; i++)
    {
        robot.receive();
        size_t call = robot.get_call() - call0;
        robot.set_target_position(franka_timeout_handler::default_target_position + 0.1 * Eigen::Matrix<double, 3, 1>(0.0, sin(2 * M_PI * call / period), cos(2 * M_PI * call / period)));
        robot.send();
        std::cout << (robot.get_late() ? "Late" : "Not late") << std::endl;
        if (robot.get_late()) late++;
    }
    std::cout << "Total: late " << late << " times of " << time << " (" << (float)late/time << "%)" << std::endl;
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}