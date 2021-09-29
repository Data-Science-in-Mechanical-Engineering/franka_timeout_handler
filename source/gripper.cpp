#include "../include/franka_timeout_handler/gripper.h"

franka_timeout_handler::Gripper::Gripper(std::string ip)
{
    _gripper = new franka::Gripper(ip);
}

bool franka_timeout_handler::Gripper::homing()
{
    return _gripper->homing();
}

double franka_timeout_handler::Gripper::get_width()
{
    return _gripper->readOnce().width;
}

bool franka_timeout_handler::Gripper::get_grasped()
{
    return _gripper->readOnce().is_grasped;
}

double franka_timeout_handler::Gripper::get_temperature()
{
    return _gripper->readOnce().temperature;
}

bool franka_timeout_handler::Gripper::move(double width, double speed)
{
    return _gripper->move(width, speed);
}

bool franka_timeout_handler::Gripper::grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
{
    return _gripper->grasp(width, speed, force, epsilon_inner, epsilon_outer);
}

bool franka_timeout_handler::Gripper::async_started()
{
    return _started;
}

void franka_timeout_handler::Gripper::async_wait()
{
    if (_thread.joinable()) _thread.join();
}

void franka_timeout_handler::Gripper::async_move(double width, double speed)
{
    _started = true;
    _gripper->stop();
    if (_thread.joinable()) _thread.join();
    Gripper *gripper = this;
    _thread = std::thread([gripper, width, speed]() -> void
    {
        try
        {
            gripper->_gripper->move(width, speed);
        }
        catch(...) {}
        gripper->_started = false;
    });
}

void franka_timeout_handler::Gripper::async_grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
{
    _started = true;
    _gripper->stop();
    if (_thread.joinable()) _thread.join();
    Gripper *gripper = this;
    _thread = std::thread([gripper, width, speed, force, epsilon_inner, epsilon_outer]() -> void
    {
        try
        {
            gripper->_gripper->grasp(width, speed, force, epsilon_inner, epsilon_outer);
        }
        catch(...) {}
        gripper->_started = false;
    });
}

franka_timeout_handler::Gripper::~Gripper()
{
    try
    {
        _gripper->stop();
    }
    catch(...) {}
    if (_thread.joinable()) _thread.join();
    delete _gripper;
}