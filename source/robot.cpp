#include "../include/franka_real_time/robot.h"

franka_real_time::Robot::Robot(std::string ip) : _robot(ip)
{
    _model = new franka::Model(_robot.loadModel());
}

franka_real_time::Robot::~Robot()
{
    delete _model;
}