#include "../include/franka_real_time/robot.h"
#include <iostream>

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./hold IP" << std::endl;
        return 1;
    }

    franka_real_time::Robot robot(argv[1]);
    robot.start();
    
    robot.receive();
    robot.set_target_position(robot.get_position());
    robot.set_target_orientation(robot.get_orientation());
    
    while (true)
    {
        robot.receive();
        robot.send();
        std::cout << (robot.get_late() ? "Late" : "Not late") << std::endl;
    }

    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}