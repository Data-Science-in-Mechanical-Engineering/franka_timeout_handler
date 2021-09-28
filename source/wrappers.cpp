#include "../include/franka_timeout_handler/robot.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(franka_timeout_handler, m)
{
    //update.h
    pybind11::enum_<franka_timeout_handler::Update>(m, "Update")
    .value("never", franka_timeout_handler::Update::never)
    .value("always", franka_timeout_handler::Update::always)
    .value("if_not_late", franka_timeout_handler::Update::if_not_late);

    //controller_type.h
    pybind11::enum_<franka_timeout_handler::ControllerType>(m, "ControllerType")
    .value("cartesian", franka_timeout_handler::ControllerType::cartesian)
    .value("joint", franka_timeout_handler::ControllerType::joint);

    //robot_core.h
    pybind11::class_<franka_timeout_handler::RobotCore>(m, "RobotCore")
        .def(pybind11::init<std::string>())
        .def("start",                               &franka_timeout_handler::Robot::start)
        .def("stop",                                &franka_timeout_handler::Robot::stop)
		.def("started",                             &franka_timeout_handler::Robot::started)
		.def("typ",                                 &franka_timeout_handler::Robot::typ)
		.def("receive",                             &franka_timeout_handler::Robot::receive)
		.def("send",                                &franka_timeout_handler::Robot::send)
		.def("receive_and_send",                    &franka_timeout_handler::Robot::receive_and_send)
		.def("send_and_receive",                    &franka_timeout_handler::Robot::send_and_receive)

        .def("get_joint_positions",                 &franka_timeout_handler::Robot::get_joint_positions)
        .def("get_joint_velocities",                &franka_timeout_handler::Robot::get_joint_velocities)
		.def("get_position",                        &franka_timeout_handler::Robot::get_position)
        .def("get_orientation",                     &franka_timeout_handler::Robot::get_orientation)
        .def("get_orientation_euler",               &franka_timeout_handler::Robot::get_orientation_euler)
        .def("get_orientation_wxyz",                &franka_timeout_handler::Robot::get_orientation_wxyz)
		.def("get_velocity",                        &franka_timeout_handler::Robot::get_velocity)
        .def("get_rotation",                        &franka_timeout_handler::Robot::get_rotation)
        .def("get_call",                            &franka_timeout_handler::Robot::get_call)
        
        .def("set_timeout",                         &franka_timeout_handler::Robot::set_timeout)
        .def("set_joint_torques_limit",             &franka_timeout_handler::Robot::get_joint_torques_limit)
        .def("set_frequency_divider",               &franka_timeout_handler::Robot::get_frequency_divider)
        .def("set_target_position",                 &franka_timeout_handler::Robot::set_target_position)
        .def("set_target_orientation",              &franka_timeout_handler::Robot::set_target_orientation)
        .def("set_target_orientation_euler",        &franka_timeout_handler::Robot::set_target_orientation_euler)
        .def("set_target_orientation_wxyz",         &franka_timeout_handler::Robot::set_target_orientation_wxyz)
        .def("set_translation_stiffness",           &franka_timeout_handler::Robot::set_translation_stiffness)
        .def("set_rotation_stiffness",              &franka_timeout_handler::Robot::set_rotation_stiffness)
        .def("set_translation_damping",             &franka_timeout_handler::Robot::set_translation_damping)
        .def("set_rotation_damping",                &franka_timeout_handler::Robot::set_rotation_damping)
        .def("set_control_rotation",                &franka_timeout_handler::Robot::set_control_rotation)
        .def("set_target_joint_positions",          &franka_timeout_handler::Robot::set_target_joint_positions)
        .def("set_joint_stiffness",                 &franka_timeout_handler::Robot::set_joint_stiffness)
        .def("set_joint_damping",                   &franka_timeout_handler::Robot::set_joint_damping)
        .def("get_timeout",                         &franka_timeout_handler::Robot::get_timeout)
        .def("get_joint_torques_limit",             &franka_timeout_handler::Robot::get_joint_torques_limit)
        .def("get_frequency_divider",               &franka_timeout_handler::Robot::get_frequency_divider)
        .def("get_target_position",                 &franka_timeout_handler::Robot::get_target_position)
        .def("get_target_orientation",              &franka_timeout_handler::Robot::get_target_orientation)
        .def("get_target_orientation_euler",        &franka_timeout_handler::Robot::get_target_orientation_euler)
        .def("get_target_orientation_wxyz",         &franka_timeout_handler::Robot::get_target_orientation_wxyz)
        .def("get_translation_stiffness",           &franka_timeout_handler::Robot::get_translation_stiffness)
        .def("get_rotation_stiffness",              &franka_timeout_handler::Robot::get_rotation_stiffness)
        .def("get_translation_damping",             &franka_timeout_handler::Robot::get_translation_damping)
        .def("get_rotation_damping",                &franka_timeout_handler::Robot::get_rotation_damping)
        .def("get_control_rotation",                &franka_timeout_handler::Robot::get_control_rotation)
        .def("get_target_joint_positions",          &franka_timeout_handler::Robot::get_target_joint_positions)
        .def("get_joint_stiffness",                 &franka_timeout_handler::Robot::get_joint_stiffness)
        .def("get_joint_damping",                   &franka_timeout_handler::Robot::get_joint_damping)
        .def("set_timeout_update",                  &franka_timeout_handler::Robot::set_timeout_update)
        .def("set_joint_torques_limit_update",      &franka_timeout_handler::Robot::set_joint_torques_limit_update)
        .def("set_frequency_divider_update",        &franka_timeout_handler::Robot::set_frequency_divider_update)
        .def("set_target_position_update",          &franka_timeout_handler::Robot::set_target_position_update)
        .def("set_target_orientation_update",       &franka_timeout_handler::Robot::set_target_orientation_update)
        .def("set_translation_stiffness_update",    &franka_timeout_handler::Robot::set_translation_stiffness_update)
        .def("set_rotation_stiffness_update",       &franka_timeout_handler::Robot::set_rotation_stiffness_update)
        .def("set_translation_damping_update",      &franka_timeout_handler::Robot::set_translation_damping_update)
        .def("set_rotation_damping_update",         &franka_timeout_handler::Robot::set_rotation_damping_update)
        .def("set_control_rotation_update",         &franka_timeout_handler::Robot::set_control_rotation_update)
        .def("set_target_joint_positions_update",   &franka_timeout_handler::Robot::set_target_joint_positions_update)
        .def("set_joint_stiffness_update",          &franka_timeout_handler::Robot::set_joint_stiffness_update)
        .def("set_joint_damping_update",            &franka_timeout_handler::Robot::set_joint_damping_update)
        .def("get_timeout_update",                  &franka_timeout_handler::Robot::get_timeout_update)
        .def("get_joint_torques_limit_update",      &franka_timeout_handler::Robot::get_joint_torques_limit_update)
        .def("get_frequency_divider_update",        &franka_timeout_handler::Robot::get_frequency_divider_update)
        .def("get_target_position_update",          &franka_timeout_handler::Robot::get_target_position_update)
        .def("get_target_orientation_update",       &franka_timeout_handler::Robot::get_target_orientation_update)
        .def("get_translation_stiffness_update",    &franka_timeout_handler::Robot::get_translation_stiffness_update)
        .def("get_rotation_stiffness_update",       &franka_timeout_handler::Robot::get_rotation_stiffness_update)
        .def("get_translation_damping_update",      &franka_timeout_handler::Robot::get_translation_damping_update)
        .def("get_rotation_damping_update",         &franka_timeout_handler::Robot::get_rotation_damping_update)
        .def("get_control_rotation_update",         &franka_timeout_handler::Robot::get_control_rotation_update)
        .def("get_target_joint_positions_update",   &franka_timeout_handler::Robot::get_target_joint_positions_update)
        .def("get_joint_stiffness_update",          &franka_timeout_handler::Robot::get_joint_stiffness_update)
        .def("get_joint_damping_update",            &franka_timeout_handler::Robot::get_joint_damping_update)
        
        .def("get_joint_torques",                   &franka_timeout_handler::Robot::get_joint_torques)
        .def("get_late",                            &franka_timeout_handler::Robot::get_late)
        .def("set_joint_torques_update",            &franka_timeout_handler::Robot::set_joint_torques_update)
        .def("get_joint_torques_update",            &franka_timeout_handler::Robot::get_joint_torques_update);

    //robot.h
    pybind11::class_<franka_timeout_handler::Robot, franka_timeout_handler::RobotCore>(m, "Robot")
        .def(pybind11::init<std::string>())
        .def("set_translation_impedance",   &franka_timeout_handler::Robot::set_translation_impedance)
        .def("get_translation_impedance",   &franka_timeout_handler::Robot::get_translation_impedance)
        .def("set_rotation_impedance",      &franka_timeout_handler::Robot::set_rotation_impedance)
        .def("get_rotation_impedance",      &franka_timeout_handler::Robot::get_rotation_impedance)
        .def("set_joint_impedance",         &franka_timeout_handler::Robot::set_joint_impedance)
        .def("get_joint_impedance",         &franka_timeout_handler::Robot::get_joint_impedance)
        .def("move_target_position",        &franka_timeout_handler::Robot::move_target_position)
        .def("move_target_position_euler",  &franka_timeout_handler::Robot::move_target_position_euler)
        .def("move_target_position_wxyz",   &franka_timeout_handler::Robot::move_target_position_wxyz)
        .def("set_update",                  &franka_timeout_handler::Robot::set_update)
        .def("set_default_output",          &franka_timeout_handler::Robot::set_default_output)
        .def("set_default_parameters",      &franka_timeout_handler::Robot::set_default_parameters)
        .def("set_default_targets",         &franka_timeout_handler::Robot::set_default_targets)
        .def("set_current_targets",         &franka_timeout_handler::Robot::set_current_targets)
        .def("distance",                    &franka_timeout_handler::Robot::distance);

    //constants.h
    m.def("default_timeout",                    []() -> unsigned int                    { return franka_timeout_handler::default_timeout; });
    m.def("default_torques_limit",              []() -> double                          { return franka_timeout_handler::default_torques_limit; });
    m.def("default_frequency_divider",          []() -> unsigned int                    { return franka_timeout_handler::default_frequency_divider; });
    m.def("default_target_position",            []() -> Eigen::Matrix<double, 3, 1>     { return franka_timeout_handler::default_target_position; });
    m.def("default_target_orientation",         []() -> Eigen::Quaterniond              { return franka_timeout_handler::default_target_orientation; });
    m.def("default_target_orientation_euler",   []() -> Eigen::Matrix<double, 3, 1>     { return franka_timeout_handler::default_target_orientation_euler; });
    m.def("default_target_orientation_wxyz",    []() -> Eigen::Matrix<double, 4, 1>     { return franka_timeout_handler::default_target_orientation_wxyz; });
    m.def("default_translation_stiffness",      []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_translation_stiffness; });
    m.def("default_rotation_stiffness",         []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_rotation_stiffness; });
    m.def("default_translation_damping",        []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_translation_damping; });
    m.def("default_rotation_damping",           []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_rotation_damping; });
    m.def("default_control_rotation",           []() -> bool                            { return franka_timeout_handler::default_control_rotation; });
    m.def("default_target_joint_positions",     []() -> Eigen::Matrix<double, 7, 1>     { return franka_timeout_handler::default_target_joint_positions; });
    m.def("default_joint_stiffness",            []() -> Eigen::Matrix<double, 7, 7>     { return franka_timeout_handler::default_joint_stiffness; });
    m.def("default_joint_damping",              []() -> Eigen::Matrix<double, 7, 7>     { return franka_timeout_handler::default_joint_damping; });
    m.def("default_update",                     []() -> franka_timeout_handler::Update  { return franka_timeout_handler::default_update; });
    m.def("max_joint_torque",                   []() -> Eigen::Matrix<double, 7, 1>     { return franka_timeout_handler::max_joint_torque; });
}