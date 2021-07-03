#include "../include/franka_real_time/robot.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(franka_real_time, m)
{
    //Update
    pybind11::enum_<franka_real_time::Update>(m, "Update")
    .value("no", franka_real_time::Update::no)
    .value("yes", franka_real_time::Update::yes)
    .value("if_not_late", franka_real_time::Update::if_not_late)
    .export_values();

    //Robot
    pybind11::class_<franka_real_time::Robot>(m, "Robot")
        .def(pybind11::init<std::string>())
        .def("start",                               &franka_real_time::Robot::start)
        .def("stop",                                &franka_real_time::Robot::stop)
		.def("receive",                             &franka_real_time::Robot::receive)
		.def("send",                                &franka_real_time::Robot::send)
		.def("receive_and_send",                    &franka_real_time::Robot::receive_and_send)
		.def("send_and_receive",                    &franka_real_time::Robot::send_and_receive)

        .def("get_joint_positions",                 &franka_real_time::Robot::get_joint_positions)
        .def("get_joint_velocities",                &franka_real_time::Robot::get_joint_velocities)
		.def("get_position",                        &franka_real_time::Robot::get_position)
        .def("get_orientation",                     &franka_real_time::Robot::get_orientation)
        .def("get_orientation_euler",               &franka_real_time::Robot::get_orientation_euler)
		.def("get_velocity",                        &franka_real_time::Robot::get_velocity)
        .def("get_rotation",                        &franka_real_time::Robot::get_rotation)
        
        .def("set_timeout",                         &franka_real_time::Robot::set_timeout)
        .def("set_target_position",                 &franka_real_time::Robot::set_target_position)
        .def("set_target_orientation",              &franka_real_time::Robot::set_target_orientation)
        .def("set_target_orientation_euler",        &franka_real_time::Robot::set_target_orientation_euler)
        .def("set_translation_stiffness",           &franka_real_time::Robot::set_translation_stiffness)
        .def("set_rotation_stiffness",              &franka_real_time::Robot::set_rotation_stiffness)
        .def("set_translation_damping",             &franka_real_time::Robot::set_translation_damping)
        .def("set_rotation_damping",                &franka_real_time::Robot::set_rotation_damping)
        .def("set_control_rotation",                &franka_real_time::Robot::set_control_rotation)
        .def("get_timeout",                         &franka_real_time::Robot::get_timeout)
        .def("get_target_position",                 &franka_real_time::Robot::get_target_position)
        .def("get_target_orientation",              &franka_real_time::Robot::get_target_orientation)
        .def("get_target_orientation_euler",        &franka_real_time::Robot::get_target_orientation_euler)
        .def("get_translation_stiffness",           &franka_real_time::Robot::get_translation_stiffness)
        .def("get_rotation_stiffness",              &franka_real_time::Robot::get_rotation_stiffness)
        .def("get_translation_damping",             &franka_real_time::Robot::get_translation_damping)
        .def("get_rotation_damping",                &franka_real_time::Robot::get_rotation_damping)
        .def("get_control_rotation",                &franka_real_time::Robot::get_control_rotation)
        .def("set_timeout_update",                  &franka_real_time::Robot::set_timeout_update)
        .def("set_target_position_update",          &franka_real_time::Robot::set_target_position_update)
        .def("set_target_orientation_update",       &franka_real_time::Robot::set_target_orientation_update)
        .def("set_translation_stiffness_update",    &franka_real_time::Robot::set_translation_stiffness_update)
        .def("set_rotation_stiffness_update",       &franka_real_time::Robot::set_rotation_stiffness_update)
        .def("set_translation_damping_update",      &franka_real_time::Robot::set_translation_damping_update)
        .def("set_rotation_damping_update",         &franka_real_time::Robot::set_rotation_damping_update)
        .def("set_control_rotation_update",         &franka_real_time::Robot::set_control_rotation_update)
        .def("get_timeout_update",                  &franka_real_time::Robot::get_timeout_update)
        .def("get_target_position_update",          &franka_real_time::Robot::get_target_position_update)
        .def("get_target_orientation_update",       &franka_real_time::Robot::get_target_orientation_update)
        .def("get_translation_stiffness_update",    &franka_real_time::Robot::get_translation_stiffness_update)
        .def("get_rotation_stiffness_update",       &franka_real_time::Robot::get_rotation_stiffness_update)
        .def("get_translation_damping_update",      &franka_real_time::Robot::get_translation_damping_update)
        .def("get_rotation_damping_update",         &franka_real_time::Robot::get_rotation_damping_update)
        .def("get_control_rotation_update",         &franka_real_time::Robot::get_control_rotation_update)
        
        .def("get_joint_torques",                   &franka_real_time::Robot::get_joint_torques)
        .def("get_late",                            &franka_real_time::Robot::get_late)
        .def("set_joint_torques_update",            &franka_real_time::Robot::set_joint_torques_update)
        .def("get_joint_torques_update",            &franka_real_time::Robot::get_joint_torques_update)
        
        .def("set_update",                          &franka_real_time::Robot::set_update)
        .def("set_default",                         &franka_real_time::Robot::set_default)
        .def("distance",                            &franka_real_time::Robot::distance)
        .def("loop",                                &franka_real_time::Robot::loop);
}