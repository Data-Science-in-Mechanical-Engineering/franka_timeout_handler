#include "../include/franka_timeout_handler/robot.h"
#include "../include/franka_timeout_handler/constants.h"
#include "../include/franka_timeout_handler/cartesian_controller.h"
#include "../include/franka_timeout_handler/joint_controller.h"
#include <stdexcept>

void franka_timeout_handler::Robot::set_translation_impedance(const Eigen::Matrix<double, 3, 1> &impedance)
{
    set_translation_stiffness(Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0));
    set_translation_damping(Eigen::DiagonalMatrix<double, 3>((2.0 * impedance.array().sqrt()).matrix()));
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::Robot::get_translation_impedance() const
{
    return get_translation_stiffness().diagonal();
}

void franka_timeout_handler::Robot::set_rotation_impedance(const Eigen::Matrix<double, 3, 1> &impedance)
{
    set_rotation_stiffness(Eigen::DiagonalMatrix<double, 3>(impedance));
    set_rotation_damping(Eigen::DiagonalMatrix<double, 3>((2.0 * impedance.array().sqrt()).matrix()));
}

Eigen::Matrix<double, 3, 1> franka_timeout_handler::Robot::get_rotation_impedance() const
{
    return get_rotation_stiffness().diagonal();
}

void franka_timeout_handler::Robot::set_joint_impedance(const Eigen::Matrix<double, 7, 1> &impedance)
{
    set_joint_stiffness(Eigen::DiagonalMatrix<double, 7>(impedance));
    set_joint_damping(Eigen::DiagonalMatrix<double, 7>((2.0 * impedance.array().sqrt()).matrix()));
}

Eigen::Matrix<double, 7, 1> franka_timeout_handler::Robot::get_joint_impedance() const
{
    return get_joint_stiffness().diagonal();
}

void franka_timeout_handler::Robot::move_target_position(const Eigen::Matrix<double, 3, 1> &position, const Eigen::Quaterniond &orientation, unsigned int time)
{
    receive();
    Eigen::Matrix<double, 3, 1> old_position = get_target_position();
    Eigen::Quaterniond old_orientation = get_target_orientation();
    size_t old_call = get_call();
    while (true)
    {
        size_t call = get_call();
        if (call - old_call > time) break;
        set_target_position(((double)(time - (call - old_call) - 1) / (time - 1)) * old_position + ((double)(call - old_call) / (time - 1)) * position);
        set_target_orientation(old_orientation.slerp((double)(call - old_call) / (time - 1), orientation));
        receive_and_send();
    }
    set_target_position(position);
    set_target_orientation(orientation);
    receive_and_send();
}

void franka_timeout_handler::Robot::move_target_position_euler(const Eigen::Matrix<double, 3, 1> &position, const Eigen::Matrix<double, 3, 1> &euler, unsigned int time)
{
    move_target_position(position, Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()), time);
}

void franka_timeout_handler::Robot::move_target_position_wxyz(const Eigen::Matrix<double, 3, 1> &position, const Eigen::Matrix<double, 4, 1> &wxyz, unsigned int time)
{
    move_target_position(position, Eigen::Quaterniond(wxyz(0), wxyz(1), wxyz(2), wxyz(3)), time);
}

void franka_timeout_handler::Robot::move_target_joint_positions(const Eigen::Matrix<double, 7, 1> &positions, unsigned int time)
{
    receive();
    Eigen::Matrix<double, 7, 1> old_positions = get_target_joint_positions();
    size_t old_call = get_call();
    while (true)
    {
        size_t call = get_call();
        if (call - old_call > time) break;
        set_target_joint_positions(((double)(time - (call - old_call) - 1) / (time - 1)) * old_positions + ((double)(call - old_call) / (time - 1)) * positions);
        receive_and_send();
    }
    set_target_joint_positions(positions);
    receive_and_send();
}

void franka_timeout_handler::Robot::set_update(Update update)
{
    set_joint_torques_update(update);
    set_timeout_update(update);
    set_joint_torques_limit_update(update);
    set_frequency_divider_update(update);
    set_target_position_update(update);
    set_translation_stiffness_update(update);
    set_rotation_stiffness_update(update);
    set_translation_damping_update(update);
    set_rotation_damping_update(update);
    set_control_rotation_update(update);
    set_target_joint_positions_update(update);
    set_joint_stiffness_update(update);
    set_joint_damping_update(update);
}

void franka_timeout_handler::Robot::set_default_output()
{
    set_default_parameters();
    set_default_targets();
}

void franka_timeout_handler::Robot::set_default_parameters()
{
    set_timeout(default_timeout);
    set_joint_torques_limit(default_torques_limit);
    set_frequency_divider(default_frequency_divider);
    set_translation_stiffness(default_translation_stiffness);
    set_rotation_stiffness(default_rotation_stiffness);
    set_translation_damping(default_translation_damping);
    set_rotation_damping(default_rotation_damping);
    set_control_rotation(default_control_rotation);
    set_joint_stiffness(default_joint_stiffness);
    set_joint_damping(default_joint_damping);
}

void franka_timeout_handler::Robot::set_default_targets()
{
    set_target_position(default_target_position);
    set_target_orientation(default_target_orientation);
    set_target_joint_positions(default_target_joint_positions);
}

void franka_timeout_handler::Robot::set_current_targets()
{
    receive();
    set_target_position(get_position());
    set_target_orientation(get_orientation());
    set_target_joint_positions(get_joint_positions());
}

double franka_timeout_handler::Robot::distance() const
{
    if (typ() == ControllerType::cartesian)
    {
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << get_position() - get_target_position();
        Eigen::Quaterniond orientation = get_orientation();
        if (get_target_orientation().coeffs().dot(orientation.coeffs()) < 0.0) orientation.coeffs() << -orientation.coeffs();
        Eigen::Quaterniond error_quaternion(orientation.inverse() * get_target_orientation());
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        return error.norm();
    }
    else
    {
        return (get_joint_positions() - get_target_joint_positions()).norm();
    }
}