#include "../include/franka_real_time/cartesial_controller.h"
#include <iostream>

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./move IP" << std::endl;
        return 1;
    }

    franka_real_time::Robot robot(argv[1]);
    franka_real_time::CartesialController controller(robot);
    
    controller.receive();
    controller.target_position = controller.position;
    controller.target_orientation = controller.orientation;
    controller.send();
    
    std::cout << "Type any key to stop:";
    std::cin.get();
    std::cout << std::endl;
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}