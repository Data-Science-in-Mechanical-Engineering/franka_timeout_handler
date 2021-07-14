#pragma once

#include "robot_core.h"

namespace franka_real_time
{
    ///Franka Panda robot with bells and whistles
    class Robot : public RobotCore
    {
    public:
        ///Sets translation impedances, i.e. diagonals of stiffness and damping matricies (output, cartesian only)
        void set_translation_impedance(Eigen::Matrix<double, 3, 1> impedance);
        ///Returns translation impedance, i.e. diagonal of stiffness matrix (output, cartesian only)
        Eigen::Matrix<double, 3, 1> get_translation_impedance() const;
        ///Sets rotation impedances, i.e. diagonals of stiffness and damping matricies (output, cartesian only)
        void set_rotation_impedance(Eigen::Matrix<double, 3, 1> impedance);
        ///Returns rotation impedance, i.e. diagonal of stiffness matrix (output, cartesian only)
        Eigen::Matrix<double, 3, 1> get_rotation_impedance()    const;
        ///Sets joint-space impedances, i.e. diagonals of stiffness and damping matricies (output, joint only)
        void set_joint_impedance(Eigen::Matrix<double, 7, 1> impedance);
        ///Returns joint-space impedance, i.e. diagonal of stiffness matrix (output, joint only)
        Eigen::Matrix<double, 7, 1> get_joint_impedance()       const;
        ///Sets update mode to all output and result variables
		void set_update(Update update);
        ///Sets all output values to default values
        void set_default();
        ///Sets target position to current position
        void set_current();
        ///Iterates till the robot reaches default position
        ///@param iterations Number of iterations
        void loop_to_default(unsigned int iterations);
        using RobotCore::RobotCore;
    };
}