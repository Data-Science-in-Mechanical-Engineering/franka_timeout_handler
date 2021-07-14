#include "../include/franka_real_time/joint_controller.h"
#include "../include/franka_real_time/robot_core.h"
#include <stdexcept>

void franka_real_time::JointController::_robot_output_to_output()
{
    Controller::_robot_output_to_output();
    if (_robot->_update_target_joint_positions != Update::no) _target_joint_positions = _robot->_target_joint_positions;
    if (_robot->_update_joint_stiffness != Update::no) _joint_stiffness = _robot->_joint_stiffness;
    if (_robot->_update_joint_damping != Update::no) _joint_damping = _robot->_joint_damping;
}

void franka_real_time::JointController::_calculate_result()
{
    _joint_torques = _joint_stiffness * (_target_joint_positions - _joint_positions) - _joint_damping * _joint_velocities + _coriolis;
}

void franka_real_time::JointController::_robot_output_to_late_output()
{
    Controller::_robot_output_to_late_output();
    if (_robot->_update_target_joint_positions == Update::yes) { _late_target_joint_positions = _robot->_target_joint_positions; _late_update_target_joint_positions = true; }
    if (_robot->_update_joint_stiffness == Update::yes) { _late_joint_stiffness = _robot->_joint_stiffness; _late_update_joint_stiffness = true; }
    if (_robot->_update_joint_damping == Update::yes) { _late_joint_damping = _robot->_joint_damping; _late_update_joint_damping = true; }
}

void franka_real_time::JointController::_late_output_to_output()
{
    Controller::_late_output_to_output();
    if (_late_update_target_joint_positions) { _target_joint_positions = _late_target_joint_positions; _late_update_target_joint_positions = false; }
    if (_late_update_joint_stiffness) { _joint_stiffness = _late_joint_stiffness; _late_update_joint_stiffness = false; }
    if (_late_update_joint_damping) { _joint_damping = _late_joint_damping; _late_update_joint_damping = false; }
}

bool franka_real_time::JointController::typ()
{
    return true;
}

void franka_real_time::JointController::set_default(RobotCore *robot_core)
{
    robot_core->_target_joint_positions << 0.0196095, 0.0141527, -0.0101538, -1.57401, -0.0177831, 1.55137, 0.831368;
    robot_core->_joint_stiffness.setZero();
    robot_core->_joint_stiffness.diagonal() << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    robot_core->_joint_stiffness *= 0.1;
    robot_core->_joint_damping.setZero();
    robot_core->_joint_damping.diagonal() << 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0;
    robot_core->_joint_damping *= 0.32;
}