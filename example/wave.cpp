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

    try
    {
        franka_timeout_handler::Robot robot(argv[1]);
        robot.start(franka_timeout_handler::ControllerType::cartesian);
        robot.move_target_position(franka_timeout_handler::default_target_position + 0.1 * Eigen::Matrix<double, 3, 1>(0.0, 0.0, 1.0), franka_timeout_handler::default_target_orientation, 3000);
        robot.receive();
        std::uint64_t call0 = robot.get_call();
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
        for (size_t i = 0; i < 1000; i++) robot.receive_and_send();
        std::cout << "Total: late " << late << " times of " << time << " (" << 100.0 * late / time << "%)" << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}