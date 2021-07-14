#include "../include/franka_real_time/cartesian_controller.h"
#include "../include/franka_real_time/robot_core.h"
#include <stdexcept>

void franka_real_time::CartesianController::_robot_output_to_output()
{
    Controller::_robot_output_to_output();
    if (_robot->_update_target_position != Update::no) _target_position = _robot->_target_position;
    if (_robot->_update_target_orientation != Update::no) _target_orientation = _robot->_target_orientation;
    if (_robot->_update_translation_stiffness != Update::no) _translation_stiffness = _robot->_translation_stiffness;
    if (_robot->_update_rotation_stiffness != Update::no) _rotation_stiffness = _robot->_rotation_stiffness;
    if (_robot->_update_translation_damping != Update::no) _translation_damping = _robot->_translation_damping;
    if (_robot->_update_rotation_damping != Update::no) _rotation_damping = _robot->_rotation_damping;
    if (_robot->_update_control_rotation != Update::no) _control_rotation = _robot->_control_rotation;
}

void franka_real_time::CartesianController::_calculate_result()
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
        _joint_torques << _rotation_correction * (_jacobian.transpose() * (-stiffness * error - damping * _velocity_rotation) + _coriolis);
    }
    else
    {
        _joint_torques << _jacobian.transpose() * (-stiffness * error - damping * _velocity_rotation) + _coriolis;
    }
}

void franka_real_time::CartesianController::_robot_output_to_late_output()
{
    Controller::_robot_output_to_late_output();
    if (_robot->_update_target_position == Update::yes) { _late_target_position = _robot->_target_position; _late_update_target_position = true; }
    if (_robot->_update_target_orientation == Update::yes) { _late_target_orientation = _robot->_target_orientation; _late_update_target_orientation = true; }
    if (_robot->_update_translation_stiffness == Update::yes) { _late_translation_stiffness = _robot->_translation_stiffness; _late_update_translation_stiffness = true; }
    if (_robot->_update_rotation_stiffness == Update::yes) { _late_rotation_stiffness = _robot->_rotation_stiffness; _late_update_rotation_stiffness = true; }
    if (_robot->_update_translation_damping == Update::yes) { _late_translation_damping = _robot->_translation_damping; _late_update_translation_damping = true; }
    if (_robot->_update_rotation_damping == Update::yes) { _late_rotation_damping = _robot->_rotation_damping; _late_update_rotation_damping = true; }
    if (_robot->_update_control_rotation == Update::yes) { _late_control_rotation = _robot->_control_rotation; _late_update_control_rotation = true; }    
}

bool franka_real_time::CartesianController::typ()
{
    return false;
}

void franka_real_time::CartesianController::_late_output_to_output()
{
    Controller::_late_output_to_output();
    if (_late_update_target_position) { _target_position = _late_target_position; _late_update_target_position = false; }
    if (_late_update_target_orientation) { _target_orientation = _late_target_orientation; _late_update_target_orientation = false; }
    if (_late_update_translation_stiffness) { _translation_stiffness = _late_translation_stiffness; _late_update_translation_stiffness = false; }
    if (_late_update_rotation_stiffness) { _rotation_stiffness = _late_rotation_stiffness; _late_update_rotation_stiffness = false; }
    if (_late_update_translation_damping) { _translation_damping = _late_translation_damping; _late_update_translation_damping = false; }
    if (_late_update_rotation_damping) { _rotation_damping = _late_rotation_damping; _late_update_rotation_damping = false; }
    if (_late_update_control_rotation) { _control_rotation = _late_control_rotation; _late_update_control_rotation = false; }
}

franka_real_time::CartesianController::CartesianController(RobotCore *robot_core) : Controller(robot_core)
{
    //Init rotation correction
    _rotation_correction.setZero();
    _rotation_correction.diagonal() << 0.5, 1.0, 1.0, 0.5, 0.05, 1.0, 0.05;
}

void franka_real_time::CartesianController::set_default(RobotCore *robot_core)
{
    robot_core->_target_position << 0.1, 0.0, 0.5;
    robot_core->_target_orientation = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
    const double translational_stiffness_constant = 100.0;
    const double rotational_stiffness_constant = 10.0;
    robot_core->_translation_stiffness = translational_stiffness_constant * Eigen::Matrix<double, 3, 3>::Identity();
    robot_core->_rotation_stiffness = rotational_stiffness_constant * Eigen::Matrix<double, 3, 3>::Identity();
    robot_core->_translation_damping = 2.0 * sqrt(translational_stiffness_constant) * Eigen::Matrix<double, 3, 3>::Identity();
    robot_core->_rotation_damping = 2.0 * sqrt(rotational_stiffness_constant) * Eigen::Matrix<double, 3, 3>::Identity();
    robot_core->_control_rotation = false;
}