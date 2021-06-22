#include "../include/franka_real_time/robot.h"
#include <iostream>

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./move IP" << std::endl;
        return 1;
    }

    franka_real_time::Robot robot(argv[1]);
    robot.control_cartesian();
    
    robot.receive();
    robot.target_position = robot.position;
    robot.target_orientation = robot.orientation;
    robot.send();
    
    std::cout << "Type any key to stop:";
    std::cin.get();
    std::cout << std::endl;

    robot.stop();
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}