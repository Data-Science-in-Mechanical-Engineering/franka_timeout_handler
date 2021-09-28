#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_timeout_handler
{
    ///Default timeout
    static const unsigned int default_timeout = 300;

    ///Default joint torque limit
    static const double default_torques_limit = 0.1;

    ///Default frequency divider
    static const unsigned int default_frequency_divider = 1;

    ///Default cartesian target position
    static const Eigen::Matrix<double, 3, 1> default_target_position = { 0.1, 0.0, 0.5 };

    ///Default cartesian target orientation
    static const Eigen::Quaterniond default_target_orientation = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);

    ///Default cartesian target position represented in Euler angles
    static const Eigen::Matrix<double, 3, 1> default_target_orientation_euler = Eigen::Matrix<double, 3, 1>(M_PI, 0.0, 0.0);

    ///Default cartesian target position represented as WXYZ vector
    static const Eigen::Matrix<double, 4, 1> default_target_orientation_wxyz = Eigen::Matrix<double, 4, 1>(0.0, 1.0, 0.0, 0.0);

    ///Default cartesian translational stiffness matrix
    static const Eigen::Matrix<double, 3, 3> default_translation_stiffness = 100.0 * Eigen::Matrix<double, 3, 3>::Identity();

    ///Default cartesian rotational stiffness matrix
    static const Eigen::Matrix<double, 3, 3> default_rotation_stiffness = 10.0 * Eigen::Matrix<double, 3, 3>::Identity();

    ///Default cartesian translational damping matrix
    static const Eigen::Matrix<double, 3, 3> default_translation_damping = 2 * sqrt(100) * Eigen::Matrix<double, 3, 3>::Identity();

    ///Default cartesian rotational damping matrix
    static const Eigen::Matrix<double, 3, 3> default_rotation_damping = 2 * sqrt(10) * Eigen::Matrix<double, 3, 3>::Identity();

    ///Default mode of rotation suppressionyes
    static const bool default_control_rotation = false;

    ///Raw default joint target angles
    static const double raw_default_target_joint_positions[7] = { 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, M_PI / 4 };
    ///Default joint target angles
    static const Eigen::Matrix<double, 7, 1> default_target_joint_positions(raw_default_target_joint_positions);

    ///Raw default joint stiffness matrix
    static const double raw_default_joint_stiffness[49] =
    {
        0.1 * 600.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.1 * 600.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.1 * 600.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.1 * 600.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1 * 250.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * 150.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * 50.0
    };
    ///Default joint stiffness matrix
    static const Eigen::Matrix<double, 7, 7> default_joint_stiffness(raw_default_joint_stiffness);
    
    ///Raw default joint damping matrix
    static const double raw_default_joint_damping[49] =
    {
        0.32 * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.32 * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.32 * 50.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.32 * 50.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.32 * 30.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.32 * 25.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.32 * 15.0
    };
    ///Default joint damping matrix
    static const Eigen::Matrix<double, 7, 7> default_joint_damping(raw_default_joint_stiffness);

    ///Default update policy
    static const Update default_update = Update::always;
    
    ///Raw maximal joint torques
    static const double raw_max_joint_torque[7] = { 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0 };
    ///Maximal joint torques
    static const Eigen::Matrix<double, 7, 1> max_joint_torque(raw_max_joint_torque);
}