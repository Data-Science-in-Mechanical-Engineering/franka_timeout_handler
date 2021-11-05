#include "../include/franka_timeout_handler/joint_controller.h"
#include "../include/franka_timeout_handler/robot_core.h"
#include <stdexcept>

void franka_timeout_handler::JointController::_robot_output_to_output()
{
    Controller::_robot_output_to_output();
    if (_robot->_update_target_joint_positions != Update::never) _target_joint_positions = _robot->_target_joint_positions;
    if (_robot->_update_target_joint_velocities != Update::never) _target_joint_velocities = _robot->_target_joint_velocities;
    if (_robot->_update_joint_stiffness != Update::never) _joint_stiffness = _robot->_joint_stiffness;
    if (_robot->_update_joint_damping != Update::never) _joint_damping = _robot->_joint_damping;
}

void franka_timeout_handler::JointController::_calculate_result()
{
    _joint_torques = _joint_stiffness * (_target_joint_positions - _joint_positions) - _joint_damping * (_joint_velocities - _target_joint_velocities) + _coriolis;
}

void franka_timeout_handler::JointController::_robot_output_to_late_output()
{
    Controller::_robot_output_to_late_output();
    if (_robot->_update_target_joint_positions == Update::always) { _late_target_joint_positions = _robot->_target_joint_positions; _late_update_target_joint_positions = true; }
    if (_robot->_update_target_joint_velocities == Update::always) { _late_target_joint_velocities = _robot->_target_joint_velocities; _late_update_target_joint_velocities = true; }
    if (_robot->_update_joint_stiffness == Update::always) { _late_joint_stiffness = _robot->_joint_stiffness; _late_update_joint_stiffness = true; }
    if (_robot->_update_joint_damping == Update::always) { _late_joint_damping = _robot->_joint_damping; _late_update_joint_damping = true; }
}

void franka_timeout_handler::JointController::_late_output_to_output()
{
    Controller::_late_output_to_output();
    if (_late_update_target_joint_positions) { _target_joint_positions = _late_target_joint_positions; _late_update_target_joint_positions = false; }
    if (_late_update_target_joint_velocities) { _target_joint_velocities = _late_target_joint_velocities; _late_update_target_joint_velocities = false; }
    if (_late_update_joint_stiffness) { _joint_stiffness = _late_joint_stiffness; _late_update_joint_stiffness = false; }
    if (_late_update_joint_damping) { _joint_damping = _late_joint_damping; _late_update_joint_damping = false; }
}

franka_timeout_handler::ControllerType franka_timeout_handler::JointController::typ() const
{
    return ControllerType::joint;
}

void franka_timeout_handler::JointController::start(RobotCore *robot_core)
{
    if (_started) throw std::runtime_error("franka_timeout_handler::JointController::start(): Controller was already started");
    _late_update_target_joint_positions    = false;
    _late_update_target_joint_velocities   = false;
    _late_update_joint_stiffness           = false;
    _late_update_joint_damping             = false;
    Controller::start(robot_core);
}