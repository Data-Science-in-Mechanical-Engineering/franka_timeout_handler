#include "../include/franka_timeout_handler/cartesian_controller.h"
#include "../include/franka_timeout_handler/robot_core.h"
#include <stdexcept>

void franka_timeout_handler::CartesianController::_robot_output_to_output()
{
    Controller::_robot_output_to_output();
    if (_robot->_update_target_position != Update::never) _target_position = _robot->_target_position;
    if (_robot->_update_target_orientation != Update::never) _target_orientation = _robot->_target_orientation;
    if (_robot->_update_target_velocity != Update::never) _target_velocity = _robot->_target_velocity;
    if (_robot->_update_target_rotation != Update::never) _target_rotation = _robot->_target_rotation;
    if (_robot->_update_translation_stiffness != Update::never) _translation_stiffness = _robot->_translation_stiffness;
    if (_robot->_update_rotation_stiffness != Update::never) _rotation_stiffness = _robot->_rotation_stiffness;
    if (_robot->_update_translation_damping != Update::never) _translation_damping = _robot->_translation_damping;
    if (_robot->_update_rotation_damping != Update::never) _rotation_damping = _robot->_rotation_damping;
    if (_robot->_update_control_rotation != Update::never) _control_rotation = _robot->_control_rotation;
}

void franka_timeout_handler::CartesianController::_calculate_result()
{
    Eigen::Matrix<double, 6, 6> stiffness;
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << _translation_stiffness;
    if (_control_rotation) stiffness.bottomRightCorner(3, 3) << _rotation_stiffness;
    Eigen::Matrix<double, 6, 6> damping;
    damping.setZero();
    damping.topLeftCorner(3, 3) << _translation_damping;
    if (_control_rotation) damping.bottomRightCorner(3, 3) << _rotation_damping;
    Eigen::Matrix<double, 6, 1> error;
    error.setZero();
    error.head(3) << _position - _target_position;
    if (_control_rotation)
    {
        if (_target_orientation.coeffs().dot(_orientation.coeffs()) < 0.0) _orientation.coeffs() << -_orientation.coeffs();
        Eigen::Quaterniond error_quaternion(_orientation.inverse() * _target_orientation);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -_transform.linear() * error.tail(3);
    }
    Eigen::Matrix<double, 6, 1> velocity_rotation_target;
    velocity_rotation_target.block<3,1>(0,0) = _target_velocity;
    velocity_rotation_target.block<3,1>(3,0) = _target_rotation;
    _joint_torques << _jacobian.transpose() * (-stiffness * error - damping * (_velocity_rotation - velocity_rotation_target)) + _coriolis;
}


void franka_timeout_handler::CartesianController::_robot_output_to_late_output()
{
    Controller::_robot_output_to_late_output();
    if (_robot->_update_target_position == Update::always) { _late_target_position = _robot->_target_position; _late_update_target_position = true; }
    if (_robot->_update_target_orientation == Update::always) { _late_target_orientation = _robot->_target_orientation; _late_update_target_orientation = true; }
    if (_robot->_update_target_velocity == Update::always) { _late_target_velocity = _robot->_target_velocity; _late_update_target_velocity = true; }
    if (_robot->_update_target_rotation == Update::always) { _late_target_rotation = _robot->_target_rotation; _late_update_target_rotation = true; }
    if (_robot->_update_translation_stiffness == Update::always) { _late_translation_stiffness = _robot->_translation_stiffness; _late_update_translation_stiffness = true; }
    if (_robot->_update_rotation_stiffness == Update::always) { _late_rotation_stiffness = _robot->_rotation_stiffness; _late_update_rotation_stiffness = true; }
    if (_robot->_update_translation_damping == Update::always) { _late_translation_damping = _robot->_translation_damping; _late_update_translation_damping = true; }
    if (_robot->_update_rotation_damping == Update::always) { _late_rotation_damping = _robot->_rotation_damping; _late_update_rotation_damping = true; }
    if (_robot->_update_control_rotation == Update::always) { _late_control_rotation = _robot->_control_rotation; _late_update_control_rotation = true; }    
}

void franka_timeout_handler::CartesianController::_late_output_to_output()
{
    Controller::_late_output_to_output();
    if (_late_update_target_position) { _target_position = _late_target_position; _late_update_target_position = false; }
    if (_late_update_target_orientation) { _target_orientation = _late_target_orientation; _late_update_target_orientation = false; }
    if (_late_update_target_velocity) { _target_velocity = _late_target_velocity; _late_update_target_velocity = false; }
    if (_late_update_target_rotation) { _target_rotation = _late_target_rotation; _late_update_target_rotation = false; }
    if (_late_update_translation_stiffness) { _translation_stiffness = _late_translation_stiffness; _late_update_translation_stiffness = false; }
    if (_late_update_rotation_stiffness) { _rotation_stiffness = _late_rotation_stiffness; _late_update_rotation_stiffness = false; }
    if (_late_update_translation_damping) { _translation_damping = _late_translation_damping; _late_update_translation_damping = false; }
    if (_late_update_rotation_damping) { _rotation_damping = _late_rotation_damping; _late_update_rotation_damping = false; }
    if (_late_update_control_rotation) { _control_rotation = _late_control_rotation; _late_update_control_rotation = false; }
}

franka_timeout_handler::ControllerType franka_timeout_handler::CartesianController::typ() const
{
    return ControllerType::cartesian;
}

void franka_timeout_handler::CartesianController::start(RobotCore *robot_core)
{
    if (_started) throw std::runtime_error("franka_timeout_handler::CartesianController::start(): Controller was already started");
    _late_update_target_position       = false;
    _late_update_target_orientation    = false;
    _late_update_target_velocity       = false;
    _late_update_target_rotation       = false;
    _late_update_translation_stiffness = false;
    _late_update_rotation_stiffness    = false;
    _late_update_translation_damping   = false;
    _late_update_rotation_damping      = false;
    _late_update_control_rotation      = false;
    Controller::start(robot_core);
}