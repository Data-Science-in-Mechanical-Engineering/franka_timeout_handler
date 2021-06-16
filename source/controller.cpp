#include "../include/franka_real_time/controller.h"
#include <stdexcept>

void franka_real_time::Controller::_set_controller(Robot *robot, Controller *controller)
{
    if (controller != nullptr && robot->_controller != nullptr) throw std::runtime_error("franka_real_time: Controller of Robot already exists");
    robot->_controller = controller;
}

franka::Robot *franka_real_time::Controller::_get_robot()
{
    return &_robot->_robot;
}

franka::Model *franka_real_time::Controller::_get_model()
{
    return _robot->_model;
}