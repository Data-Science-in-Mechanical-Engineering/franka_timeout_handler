#pragma once

#include "robot_core.h"

namespace franka_timeout_handler
{
    ///Franka Panda robot with bells and whistles
    class Robot : public RobotCore
    {
    public:
        ///Sets translation impedances, i.e. diagonals of stiffness and damping matricies (output, cartesian only)
        void set_translation_impedance(const Eigen::Matrix<double, 3, 1> &impedance);
        ///Returns translation impedance, i.e. diagonal of stiffness matrix (output, cartesian only)
        Eigen::Matrix<double, 3, 1> get_translation_impedance() const;
        ///Sets rotation impedances, i.e. diagonals of stiffness and damping matricies (output, cartesian only)
        void set_rotation_impedance(const Eigen::Matrix<double, 3, 1> &impedance);
        ///Returns rotation impedance, i.e. diagonal of stiffness matrix (output, cartesian only)
        Eigen::Matrix<double, 3, 1> get_rotation_impedance()    const;
        ///Sets joint-space impedances, i.e. diagonals of stiffness and damping matricies (output, joint only)
        void set_joint_impedance(const Eigen::Matrix<double, 7, 1> &impedance);
        ///Returns joint-space impedance, i.e. diagonal of stiffness matrix (output, joint only)
        Eigen::Matrix<double, 7, 1> get_joint_impedance()       const;
        ///Slowly moves cartesian target to given position
        void move_target_position(const Eigen::Matrix<double, 3, 1> &position, const Eigen::Quaterniond &orientation, unsigned int time);
        ///Slowly moves cartesian target to given position with orientation represented as Euler angles
        void move_target_position_euler(const Eigen::Matrix<double, 3, 1> &position, const Eigen::Matrix<double, 3, 1> &euler, unsigned int time);
        ///Slowly moves cartesian target to given position with orientation represented as WXYZ vector
        void move_target_position_wxyz(const Eigen::Matrix<double, 3, 1> &position, const Eigen::Matrix<double, 4, 1> &wxyz, unsigned int time);
        ///Slowly moves joint targets to given angles
        void move_target_joint_positions(const Eigen::Matrix<double, 7, 1> &positions, unsigned int time);
        ///Sets update mode to all output and result variables
		void set_update(Update update);
        ///Sets outputs (outputs except targets) to default values
        void set_default_output();
        ///Sets parameters (outputs except targets) to default values
        void set_default_parameters();
        ///Sets targets (type of outputs) to default values
        void set_default_targets();
        ///Sets target position to current position
        void set_current_targets();
        ///Estimates second norm between target and current position
        double distance() const;
        using RobotCore::RobotCore;
    };
}