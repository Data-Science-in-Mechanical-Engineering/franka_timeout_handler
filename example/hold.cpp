#include "../include/franka_timeout_handler/robot.h"
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
        const size_t time = 10000;
        size_t late = 0;
        for (size_t i = 0; i < time; i++)
        {
            robot.receive();
            robot.send();
            std::cout << (robot.get_late() ? "Late" : "Not late") << std::endl;
            if (robot.get_late()) late++;
        }
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