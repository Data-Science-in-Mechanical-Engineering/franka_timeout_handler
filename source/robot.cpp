#include "../include/franka_real_time/robot.h"
#include "../include/franka_real_time/cartesian_controller.h"
#include "../include/franka_real_time/joint_controller.h"
#include <stdexcept>

void franka_real_time::Robot::set_translation_impedance(Eigen::Matrix<double, 3, 1> impedance)
{
    _translation_stiffness.setZero();
    _translation_stiffness.diagonal() = impedance;
    _translation_damping.setZero();
    _translation_damping.diagonal() = 2.0 * impedance.array().sqrt();
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_translation_impedance() const
{
    return _translation_stiffness.diagonal();
}

void franka_real_time::Robot::set_rotation_impedance(Eigen::Matrix<double, 3, 1> impedance)
{
    _rotation_stiffness.setZero();
    _rotation_stiffness.diagonal() = impedance;
    _rotation_damping.setZero();
    _rotation_damping.diagonal() = 2.0 * impedance.array().sqrt();
}

Eigen::Matrix<double, 3, 1> franka_real_time::Robot::get_rotation_impedance() const
{
    return _rotation_stiffness.diagonal();
}

void franka_real_time::Robot::set_joint_impedance(Eigen::Matrix<double, 7, 1> impedance)
{
    _joint_stiffness.setZero();
    _joint_stiffness.diagonal() = impedance;
    _joint_damping.setZero();
    _joint_damping.diagonal() = 2.0 * impedance.array().sqrt();
}

Eigen::Matrix<double, 7, 1> franka_real_time::Robot::get_joint_impedance() const
{
    return _joint_stiffness.diagonal();
}

void franka_real_time::Robot::set_update(Update update)
{
    _update_joint_torques = update;
    _update_timeout = update;
    _update_joint_torques_limit = update;
    _update_frequency_divider = update;
    _update_target_position = update;
    _update_translation_stiffness = update;
    _update_rotation_stiffness = update;
    _update_translation_damping = update;
    _update_rotation_damping = update;
    _update_control_rotation = update;
    _update_target_joint_positions = update;
    _update_joint_stiffness = update;
    _update_joint_damping = update;
}

void franka_real_time::Robot::set_default()
{
    Controller::set_default(this);
    CartesianController::set_default(this);
    JointController::set_default(this);
}

void franka_real_time::Robot::set_current()
{
    receive();
    _target_position = _position;
    _target_orientation = _orientation;
    _target_joint_positions = _joint_positions;
}

void franka_real_time::Robot::loop_to_default(unsigned int iterations)
{
    if (_controller == nullptr) throw std::runtime_error("franka_real_time: controller was not started");
    receive();
    set_default();
    if (_controller->typ())
    {
        Eigen::Matrix<double, 7, 1> initial_target = _joint_positions;
        Eigen::Matrix<double, 7, 1> final_target = _target_joint_positions;
        for (unsigned int i = 0; i < iterations; i++)
        {
            _target_joint_positions = (initial_target * (iterations - i - 1) + final_target * i) / (iterations - 1);
            receive_and_send();
        }
    }
    else
    {
        Eigen::Matrix<double, 3, 1> initial_position_target = _position;
        Eigen::Quaterniond initial_orientation_target = _orientation;
        Eigen::Matrix<double, 3, 1> final_position_target = _target_position;
        Eigen::Quaterniond final_orientation_target = _target_orientation;
        for (unsigned int i = 0; i < iterations; i++)
        {
            _target_position = (initial_position_target * (iterations - i - 1) + final_position_target * i) / (iterations - 1);
            _target_orientation = initial_orientation_target.slerp((double) i / (iterations - 1), final_orientation_target);
            receive_and_send();
        }    
    }
    for (unsigned int i = 0; i < iterations / 5; i++)
    {
        receive_and_send();
    }
}