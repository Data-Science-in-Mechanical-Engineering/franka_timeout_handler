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
        .def_readwrite("joint_positions",               &franka_real_time::Robot::joint_positions)
        .def_readwrite("joint_velocities",              &franka_real_time::Robot::joint_velocities)
		.def_readwrite("position",                      &franka_real_time::Robot::position)
        .def_readwrite("orientation",                   &franka_real_time::Robot::orientation)
        .def_readwrite("velocity",                      &franka_real_time::Robot::velocity)
        .def_readwrite("rotation",                      &franka_real_time::Robot::rotation)
		
        .def_readwrite("timeout",                       &franka_real_time::Robot::timeout)
        .def_readwrite("target_position",               &franka_real_time::Robot::target_position)
        .def_readwrite("target_orientation",            &franka_real_time::Robot::target_orientation)
        .def_readwrite("translation_stiffness",         &franka_real_time::Robot::translation_stiffness)
        .def_readwrite("rotation_stiffness",            &franka_real_time::Robot::rotation_stiffness)
        .def_readwrite("translation_damping",           &franka_real_time::Robot::translation_damping)
        .def_readwrite("rotation_damping",              &franka_real_time::Robot::rotation_damping)
        .def_readwrite("control_rotation",              &franka_real_time::Robot::control_rotation)
        
        .def_readwrite("timeout_update",                &franka_real_time::Robot::timeout_update)
        .def_readwrite("target_position_update",        &franka_real_time::Robot::target_position_update)
        .def_readwrite("target_orientation_update",     &franka_real_time::Robot::target_orientation_update)
        .def_readwrite("translation_stiffness_update",  &franka_real_time::Robot::translation_stiffness_update)
        .def_readwrite("rotation_stiffness_update",     &franka_real_time::Robot::rotation_stiffness_update)
        .def_readwrite("translation_damping_update",    &franka_real_time::Robot::translation_damping_update)
        .def_readwrite("rotation_damping_update",       &franka_real_time::Robot::rotation_damping_update)
        .def_readwrite("control_rotation_update",       &franka_real_time::Robot::control_rotation_update)

		.def_readwrite("joint_torques",                 &franka_real_time::Robot::joint_torques)
        .def_readwrite("joint_torques_update",          &franka_real_time::Robot::joint_torques_update)
        .def_readwrite("late",                          &franka_real_time::Robot::late)
        
        .def(pybind11::init<std::string>())
        .def("control_cartesian",                       &franka_real_time::Robot::control_cartesian)
		.def("update",                                  &franka_real_time::Robot::update)
		.def("receive",                                 &franka_real_time::Robot::receive)
		.def("send",                                    &franka_real_time::Robot::send)
		.def("receive_and_send",                        &franka_real_time::Robot::receive_and_send)
		.def("send_and_receive",                        &franka_real_time::Robot::send_and_receive)
        .def("distance",                                &franka_real_time::Robot::distance)
        .def("stop",                                    &franka_real_time::Robot::stop);
}