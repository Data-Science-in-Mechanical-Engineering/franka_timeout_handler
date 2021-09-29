#include "../include/franka_timeout_handler/robot.h"
#include "../include/franka_timeout_handler/gripper.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(franka_timeout_handler, m)
{
    m.doc() = "franka_timeout_handler " + std::to_string(franka_o80::version_major) + "." + std::to_string(franka_o80::version_minor) + "." + std::to_string(franka_o80::version_patch) +
    " is a library for Franka Emika Panda robot, which allows you to send messages to real-time controller from non-real-time application without breaking robot's real-time requirements";

    //constants.h
    m.def("default_timeout",                    []() -> unsigned int                    { return franka_timeout_handler::default_timeout; },                    "Default timeout in microseconds");
    m.def("default_torques_limit",              []() -> double                          { return franka_timeout_handler::default_torques_limit; },              "Default joint torque limit");
    m.def("default_frequency_divider",          []() -> unsigned int                    { return franka_timeout_handler::default_frequency_divider; },          "Default frequency divider");
    m.def("default_target_position",            []() -> Eigen::Matrix<double, 3, 1>     { return franka_timeout_handler::default_target_position; },            "Default cartesian target position");
    m.def("default_target_orientation",         []() -> Eigen::Quaterniond              { return franka_timeout_handler::default_target_orientation; },         "Default cartesian target orientation");
    m.def("default_target_orientation_euler",   []() -> Eigen::Matrix<double, 3, 1>     { return franka_timeout_handler::default_target_orientation_euler; },   "Default cartesian target position represented in Euler angles");
    m.def("default_target_orientation_wxyz",    []() -> Eigen::Matrix<double, 4, 1>     { return franka_timeout_handler::default_target_orientation_wxyz; },    "Default cartesian target position represented as WXYZ vector");
    m.def("default_translation_stiffness",      []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_translation_stiffness; },      "Default cartesian translational stiffness matrix");
    m.def("default_rotation_stiffness",         []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_rotation_stiffness; },         "Default cartesian rotational stiffness matrix");
    m.def("default_translation_damping",        []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_translation_damping; },        "Default cartesian translational damping matrix");
    m.def("default_rotation_damping",           []() -> Eigen::Matrix<double, 3, 3>     { return franka_timeout_handler::default_rotation_damping; },           "Default cartesian rotational damping matrix");
    m.def("default_control_rotation",           []() -> bool                            { return franka_timeout_handler::default_control_rotation; },           "Default mode of orientation control");
    m.def("default_target_joint_positions",     []() -> Eigen::Matrix<double, 7, 1>     { return franka_timeout_handler::default_target_joint_positions; },     "Default joint target angles");
    m.def("default_joint_stiffness",            []() -> Eigen::Matrix<double, 7, 7>     { return franka_timeout_handler::default_joint_stiffness; },            "Default joint stiffness matrix");
    m.def("default_joint_damping",              []() -> Eigen::Matrix<double, 7, 7>     { return franka_timeout_handler::default_joint_damping; },              "Default joint damping matrix");
    m.def("default_update",                     []() -> franka_timeout_handler::Update  { return franka_timeout_handler::default_update; },                     "Default update policy");
    m.def("max_joint_torque",                   []() -> Eigen::Matrix<double, 7, 1>     { return franka_timeout_handler::max_joint_torque; },                   "Maximal joint torques");

    //controller_type.h
    pybind11::enum_<franka_timeout_handler::ControllerType>(m, "ControllerType","Type of the controller")
        .value("cartesian", franka_timeout_handler::ControllerType::cartesian,  "Makes robot try to approach given cartesian position")
        .value("joint", franka_timeout_handler::ControllerType::joint,          "Makes robot try to approach given joint positions");

    //gripper.h
    pybind11::class_<franka_timeout_handler::Gripper>(m, "Gripper",                 "Simple class for robot's gripper. It is made for reasons of completenes and autonomy, and is not capable of any timeout handling.")
        .def(pybind11::init<std::string>(),                                         "Creates gripper")
        .def("homing",          &franka_timeout_handler::Gripper::homing,           "Calibrates gripper's fingers")
        .def("get_width",       &franka_timeout_handler::Gripper::get_width,        "Returns fingers' width")
        .def("get_grasped",     &franka_timeout_handler::Gripper::get_grasped,      "Returns fingers have grasped object, i.e. (width - epsillon_inner) < real_width < (width + epsillon_outer)")
        .def("get_temperature", &franka_timeout_handler::Gripper::get_temperature,  "Returns fingers' temperature")
        .def("move",            &franka_timeout_handler::Gripper::move,             pybind11::arg("width"), pybind11::arg("speed"), "Moves the fingers to a specified width with specified speed")
        .def("grasp",           &franka_timeout_handler::Gripper::grasp,            pybind11::arg("width"), pybind11::arg("speed"), pybind11::arg("force"), pybind11::arg("epsilon_inner") = 0.005, pybind11::arg("epsilon_outer") = 0.005, "Opens or closes fingers with specified force and speed")
        .def("async_started",   &franka_timeout_handler::Gripper::async_started,    "Returns if asyncronous comand is currently running")
        .def("async_wait",      &franka_timeout_handler::Gripper::async_wait,       "Waits completiting of asyncronous command")
        .def("async_move",      &franka_timeout_handler::Gripper::async_move,       pybind11::arg("width"),   pybind11::arg("speed"), "Moves the fingers to a specified width with specified speed in asyncronous mode")
        .def("async_grasp",     &franka_timeout_handler::Gripper::async_grasp,      pybind11::arg("width"),  pybind11::arg("speed"), pybind11::arg("force"), pybind11::arg("epsilon_inner") = 0.005, pybind11::arg("epsilon_outer") = 0.005, "Opens or closes fingers with specified force and speed in asyncronous mode");

    //robot_core.h
    pybind11::class_<franka_timeout_handler::RobotCore>(m, "RobotCore")
        .def(pybind11::init<std::string>(),                                                             "Creates robot")
        .def("start",                               &franka_timeout_handler::Robot::start,              "Starts controller")
        .def("stop",                                &franka_timeout_handler::Robot::stop,               "Stops controller")
        .def("started",                             &franka_timeout_handler::Robot::started,            "Returns if controller is started")
        .def("typ",                                 &franka_timeout_handler::Robot::typ,                "Returns type of controller")
        .def("receive",                             &franka_timeout_handler::Robot::receive,            "Waits for next signal (if controller is running) and refreshes inputs")
        .def("send",                                &franka_timeout_handler::Robot::send,               "Sends signal back, updates outputs, refreshes results")
        .def("receive_and_send",                    &franka_timeout_handler::Robot::receive_and_send,   "Waits for signal and immediately sends signal back with no chance to be late, refreshes inputs and results, updates outputs")
        .def("send_and_receive",                    &franka_timeout_handler::Robot::send_and_receive,   "Sends signal back and waits for new signal, updates outputs, refreshes results and then inputs")

        .def("get_joint_positions",                 &franka_timeout_handler::Robot::get_joint_positions,    "Returns joint positions (input)")
        .def("get_joint_velocities",                &franka_timeout_handler::Robot::get_joint_velocities,   "Returns joint velocities (input)")
        .def("get_position",                        &franka_timeout_handler::Robot::get_position,           "Returns cartesian position (input)")
        .def("get_orientation",                     &franka_timeout_handler::Robot::get_orientation,        "Returns cartesian orientation (input)")
        .def("get_orientation_euler",               &franka_timeout_handler::Robot::get_orientation_euler,  "Returns cartesian orientation in Euler angles: yaw, pitch, roll (input)")
        .def("get_orientation_wxyz",                &franka_timeout_handler::Robot::get_orientation_wxyz,   "Returns cartesian orientation as quanterion, but in XYZW form (output)")
        .def("get_velocity",                        &franka_timeout_handler::Robot::get_velocity,           "Returns cartesian velocity (input)")
        .def("get_rotation",                        &franka_timeout_handler::Robot::get_rotation,           "Returns cartesian rotation (input)")
        .def("get_call",                            &franka_timeout_handler::Robot::get_call,               "Returns call number, call happens every millisecond (input)")
        
        .def("set_timeout",                         &franka_timeout_handler::Robot::set_timeout,                        "Sets timeout in microsencods (output)")
        .def("set_joint_torques_limit",             &franka_timeout_handler::Robot::set_joint_torques_limit,            "Sets security limit for torques, 1.0 to full torques (output)")
        .def("set_frequency_divider",               &franka_timeout_handler::Robot::set_frequency_divider,              "Sets frequency divider (output)")
        .def("set_target_position",                 &franka_timeout_handler::Robot::set_target_position,                "Sets cartesian position of target (output)")
        .def("set_target_orientation",              &franka_timeout_handler::Robot::set_target_orientation,             "Sets cartesian orientation of tartget (output)")
        .def("set_target_orientation_euler",        &franka_timeout_handler::Robot::set_target_orientation_euler,       "Sets cartesian orientation of target in Euler angles: yaw, pitch, roll (output)")
        .def("set_target_orientation_wxyz",         &franka_timeout_handler::Robot::set_target_orientation_wxyz,        "Sets cartesian orientation of target as quanterion, but in XYZW form (output)")
        .def("set_translation_stiffness",           &franka_timeout_handler::Robot::set_translation_stiffness,          "Sets translation stiffness matrix (output)")
        .def("set_rotation_stiffness",              &franka_timeout_handler::Robot::set_rotation_stiffness,             "Sets rotation stiffness matrix (output)")
        .def("set_translation_damping",             &franka_timeout_handler::Robot::set_translation_damping,            "Sets translation damping matrix (output)")
        .def("set_rotation_damping",                &franka_timeout_handler::Robot::set_rotation_damping,               "Sets rotation damping matrix (output)")
        .def("set_control_rotation",                &franka_timeout_handler::Robot::set_control_rotation,               "Sets indicator if orientation should be controller (output)")
        .def("set_target_joint_positions",          &franka_timeout_handler::Robot::set_target_joint_positions,         "Sets joint-space target (output)")
        .def("set_joint_stiffness",                 &franka_timeout_handler::Robot::set_joint_stiffness,                "Sets joint-space stiffness matrix (output)")
        .def("set_joint_damping",                   &franka_timeout_handler::Robot::set_joint_damping,                  "Sets joint-space damping matrix (output)")
        .def("get_timeout",                         &franka_timeout_handler::Robot::get_timeout,                        "Returns timeout in microsencods (output)")
        .def("get_joint_torques_limit",             &franka_timeout_handler::Robot::get_joint_torques_limit,            "Returns security limit for torques (output)")
        .def("get_frequency_divider",               &franka_timeout_handler::Robot::get_frequency_divider,              "Returns frequency divider (output)")
        .def("get_target_position",                 &franka_timeout_handler::Robot::get_target_position,                "Returns cartesian position of tartget (output)")
        .def("get_target_orientation",              &franka_timeout_handler::Robot::get_target_orientation,             "Returns cartesian orientation of tartget (output)")
        .def("get_target_orientation_euler",        &franka_timeout_handler::Robot::get_target_orientation_euler,       "Returns cartesian orientation of target in Euler angles: yaw, pitch, roll (output)")
        .def("get_target_orientation_wxyz",         &franka_timeout_handler::Robot::get_target_orientation_wxyz,        "Returns cartesian orientation of target as quanterion, but in XYZW form (output)")
        .def("get_translation_stiffness",           &franka_timeout_handler::Robot::get_translation_stiffness,          "Returns translation stiffness matrix (output)")
        .def("get_rotation_stiffness",              &franka_timeout_handler::Robot::get_rotation_stiffness,             "Returns rotation stiffness matrix (output)")
        .def("get_translation_damping",             &franka_timeout_handler::Robot::get_translation_damping,            "Returns translation damping matrix (output)")
        .def("get_rotation_damping",                &franka_timeout_handler::Robot::get_rotation_damping,               "Returns rotation damping matrix (output)")
        .def("get_control_rotation",                &franka_timeout_handler::Robot::get_control_rotation,               "Returns if orientation should be controlled (output)")
        .def("get_target_joint_positions",          &franka_timeout_handler::Robot::get_target_joint_positions,         "Returns joint-space target (output)")
        .def("get_joint_stiffness",                 &franka_timeout_handler::Robot::get_joint_stiffness,                "Returns joint-space stiffness matrix (output)")
        .def("get_joint_damping",                   &franka_timeout_handler::Robot::get_joint_damping,                  "Returns joint-space damping matrix (output)")
        .def("set_timeout_update",                  &franka_timeout_handler::Robot::set_timeout_update,                 "Sets update mode of timeout (output)")
        .def("set_joint_torques_limit_update",      &franka_timeout_handler::Robot::set_joint_torques_limit_update,     "Sets update mode of security limit for torques (output)")
        .def("set_frequency_divider_update",        &franka_timeout_handler::Robot::set_frequency_divider_update,       "Sets update mode of frequency divider (output)")
        .def("set_target_position_update",          &franka_timeout_handler::Robot::set_target_position_update,         "Sets update mode of target cartesian position (output)")
        .def("set_target_orientation_update",       &franka_timeout_handler::Robot::set_target_orientation_update,      "Sets update mode of target cartesian orientation (output)")
        .def("set_translation_stiffness_update",    &franka_timeout_handler::Robot::set_translation_stiffness_update,   "Sets update mode of translation stiffness matrix (output)")
        .def("set_rotation_stiffness_update",       &franka_timeout_handler::Robot::set_rotation_stiffness_update,      "Sets update mode of rotation stiffness matrix (output)")
        .def("set_translation_damping_update",      &franka_timeout_handler::Robot::set_translation_damping_update,     "Sets update mode of translation damping matrix (output)")
        .def("set_rotation_damping_update",         &franka_timeout_handler::Robot::set_rotation_damping_update,        "Sets update mode of rotation damping matrix (output)")
        .def("set_control_rotation_update",         &franka_timeout_handler::Robot::set_control_rotation_update,        "Sets update mode of indicator if orientation should be controlled (output)")
        .def("set_target_joint_positions_update",   &franka_timeout_handler::Robot::set_target_joint_positions_update,  "Sets update mode of joint-space target (output)")
        .def("set_joint_stiffness_update",          &franka_timeout_handler::Robot::set_joint_stiffness_update,         "Sets update mode of joint-space stiffness matrix (output)")
        .def("set_joint_damping_update",            &franka_timeout_handler::Robot::set_joint_damping_update,           "Sets update mode of joint-space damping matrix (output)")
        .def("get_timeout_update",                  &franka_timeout_handler::Robot::get_timeout_update,                 "Returns update mode of timeout (output)")
        .def("get_joint_torques_limit_update",      &franka_timeout_handler::Robot::get_joint_torques_limit_update,     "Returns update mode of security limit for torques (output)")
        .def("get_frequency_divider_update",        &franka_timeout_handler::Robot::get_frequency_divider_update,       "Returns update mode of frequency divider (output)")
        .def("get_target_position_update",          &franka_timeout_handler::Robot::get_target_position_update,         "Returns update mode of target cartesian position (output)")
        .def("get_target_orientation_update",       &franka_timeout_handler::Robot::get_target_orientation_update,      "Returns update mode of target cartesian orientation (output)")
        .def("get_translation_stiffness_update",    &franka_timeout_handler::Robot::get_translation_stiffness_update,   "Returns update mode of translation stiffness matrix (output)")
        .def("get_rotation_stiffness_update",       &franka_timeout_handler::Robot::get_rotation_stiffness_update,      "Returns update mode of rotation stiffness matrix (output)")
        .def("get_translation_damping_update",      &franka_timeout_handler::Robot::get_translation_damping_update,     "Returns update mode of translation damping matrix (output)")
        .def("get_rotation_damping_update",         &franka_timeout_handler::Robot::get_rotation_damping_update,        "Returns update mode of rotation damping matrix (output)")
        .def("get_control_rotation_update",         &franka_timeout_handler::Robot::get_control_rotation_update,        "Returns update mode of indicator if orientation should be controlled (output)")
        .def("get_target_joint_positions_update",   &franka_timeout_handler::Robot::get_target_joint_positions_update,  "Returns update mode of joint-space target (output)")
        .def("get_joint_stiffness_update",          &franka_timeout_handler::Robot::get_joint_stiffness_update,         "Returns update mode of joint-space stiffness matrix (output)")
        .def("get_joint_damping_update",            &franka_timeout_handler::Robot::get_joint_damping_update,           "Returns update mode of joint-space damping matrix (output)")
        
        .def("get_joint_torques",                   &franka_timeout_handler::Robot::get_joint_torques,          "Returns torques sent to the robot (result)")
        .def("get_late",                            &franka_timeout_handler::Robot::get_late,                   "Returns if `send()` was called too late (result)")
        .def("set_joint_torques_update",            &franka_timeout_handler::Robot::set_joint_torques_update,   "Sets update mode of torques (result)")
        .def("get_joint_torques_update",            &franka_timeout_handler::Robot::get_joint_torques_update,   "Returns update mode of torques (result)");

    //robot.h
    pybind11::class_<franka_timeout_handler::Robot, franka_timeout_handler::RobotCore>(m, "Robot",      "Franka Panda robot with extended functionality")
        .def(pybind11::init<std::string>(),                                                             "Creates robot")
        .def("set_translation_impedance",   &franka_timeout_handler::Robot::set_translation_impedance,  "Sets translation impedances, i.e. diagonals of stiffness and damping matricies (output)")
        .def("get_translation_impedance",   &franka_timeout_handler::Robot::get_translation_impedance,  "Returns translation impedance, i.e. diagonal of stiffness matrix (output)")
        .def("set_rotation_impedance",      &franka_timeout_handler::Robot::set_rotation_impedance,     "Sets rotation impedances, i.e. diagonals of stiffness and damping matricies (output)")
        .def("get_rotation_impedance",      &franka_timeout_handler::Robot::get_rotation_impedance,     "Returns rotation impedance, i.e. diagonal of stiffness matrix (output)")
        .def("set_joint_impedance",         &franka_timeout_handler::Robot::set_joint_impedance,        "Sets joint-space impedances, i.e. diagonals of stiffness and damping matricies (output)")
        .def("get_joint_impedance",         &franka_timeout_handler::Robot::get_joint_impedance,        "Returns joint-space impedance, i.e. diagonal of stiffness matrix (output)")
        .def("move_target_position",        &franka_timeout_handler::Robot::move_target_position,       "Slowly moves cartesian target to given position with orientation represented as Euler angles")
        .def("move_target_position_euler",  &franka_timeout_handler::Robot::move_target_position_euler, "Slowly moves cartesian target to given position with orientation represented as WXYZ vector")
        .def("move_target_position_wxyz",   &franka_timeout_handler::Robot::move_target_position_wxyz,  "Slowly moves joint targets to given angles")
        .def("set_update",                  &franka_timeout_handler::Robot::set_update,                 "Sets update mode to all output and result variables")
        .def("set_default_output",          &franka_timeout_handler::Robot::set_default_output,         "Sets outputs (outputs except targets) to default values")
        .def("set_default_parameters",      &franka_timeout_handler::Robot::set_default_parameters,     "Sets parameters (outputs except targets) to default values")
        .def("set_default_targets",         &franka_timeout_handler::Robot::set_default_targets,        "Sets targets (type of outputs) to default values")
        .def("set_current_targets",         &franka_timeout_handler::Robot::set_current_targets,        "Sets target position to current position")
        .def("distance",                    &franka_timeout_handler::Robot::distance,                   "Estimates second norm between target and current position");

    //update.h
    pybind11::enum_<franka_timeout_handler::Update>(m, "Update",            "Update policy of controller parameters")
        .value("never", franka_timeout_handler::Update::never,              "Parameter is never updated")
        .value("always", franka_timeout_handler::Update::always,            "If send() is not late, parameter is updated immediately. Otherwise, it is updated in next cycle")
        .value("if_not_late", franka_timeout_handler::Update::if_not_late,  "Parameter is updated only if  send() is not late");


    //version.hpp
    m.def("version_major",  []() -> int { return franka_timeout_handler::version_major; }, "Major version");
    m.def("version_minor",  []() -> int { return franka_timeout_handler::version_minor; }, "Minor version");
    m.def("version_patch",  []() -> int { return franka_timeout_handler::version_patch; }, "Patch numebr");
}