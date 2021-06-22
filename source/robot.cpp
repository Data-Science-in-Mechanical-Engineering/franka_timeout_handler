#include "../include/franka_real_time/robot.h"
#include <stdexcept>

franka_real_time::Robot::Robot(std::string ip) : _robot(ip)
{
    _model = new franka::Model(_robot.loadModel());
}

void franka_real_time::Robot::control_cartesian()
{
    _controller = new CartesialController(this);
}

void franka_real_time::Robot::update(Update upd)
{
    timeout_update = upd;
    target_position_update = upd;
    translation_stiffness_update = upd;
    rotation_stiffness_update = upd;
    translation_damping_update = upd;
    rotation_damping_update = upd;
    control_rotation_update = upd;

    joint_torques_update = upd;
}

void franka_real_time::Robot::receive()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: no controller was started");
    _controller->receive();
}

void franka_real_time::Robot::send()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: no controller was started");
    _controller->send();
}

void franka_real_time::Robot::receive_and_send()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: no controller was started");
    _controller->receive_and_send();
}

void franka_real_time::Robot::send_and_receive()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: no controller was started");
    _controller->send_and_receive();
}

double franka_real_time::Robot::distance() const
{
    Eigen::Matrix<double, 6, 1> error;
    error.setZero();

    //Position
    error.head(3) = position - target_position;

    //Orientation
    if (control_rotation)
    {
        Eigen::Quaterniond orient = orientation;
        if (target_orientation.coeffs().dot(orient.coeffs()) < 0.0) orient.coeffs() = -orient.coeffs();
        Eigen::Quaterniond orientation_error = orient.inverse() * target_orientation;
        error.tail(3) << orientation_error.x(), orientation_error.y(), orientation_error.z();
    }

    return error.norm();
}

void franka_real_time::Robot::stop()
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: no controller was started");
    delete _controller;
    _controller = nullptr;
}

franka_real_time::Robot::~Robot()
{
    if (_controller != nullptr) delete _controller;
    delete _model;
}