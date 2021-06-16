#include "../include/franka_real_time/cartesial_controller.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

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
        .def(pybind11::init<std::string>());

    //CartesialController
    pybind11::class_<franka_real_time::CartesialController>(m, "CartesialController")
        .def(pybind11::init<franka_real_time::Robot&>())
        //.def("joint_positions",             &franka_real_time::CartesialController::joint_positions)
        //.def("joint_velocities",            &franka_real_time::CartesialController::joint_velocities)
		//.def("position",                    &franka_real_time::CartesialController::position)
        //.def("orientation",                 &franka_real_time::CartesialController::orientation)
		//.def("timeout",                     &franka_real_time::CartesialController::timeout)
        //.def("target_position",             &franka_real_time::CartesialController::target_position)
        //.def("target_orientation",          &franka_real_time::CartesialController::target_orientation)
        //.def("stiffness",                   &franka_real_time::CartesialController::stiffness)
        //.def("damping",                     &franka_real_time::CartesialController::damping)
        //.def("timeout_update",              &franka_real_time::CartesialController::timeout_update)
        //.def("target_position_update",      &franka_real_time::CartesialController::target_position_update)
        //.def("target_orientation_update",   &franka_real_time::CartesialController::target_orientation_update)
        //.def("stiffness_update",            &franka_real_time::CartesialController::stiffness_update)
        //.def("damping_update",              &franka_real_time::CartesialController::damping_update)
		//.def("joint_torques",               &franka_real_time::CartesialController::joint_torques)
        //.def("joint_torques_update",        &franka_real_time::CartesialController::joint_torques_update)
        //.def("late",                        &franka_real_time::CartesialController::late)
		//.def("update",                      &franka_real_time::CartesialController::update)
		//.def("receive",                     &franka_real_time::CartesialController::receive)
		//.def("send",                        &franka_real_time::CartesialController::send)
		//.def("receive_and_send",            &franka_real_time::CartesialController::receive_and_send)
		//.def("send_and_receive",            &franka_real_time::CartesialController::send_and_receive);
        ;
}