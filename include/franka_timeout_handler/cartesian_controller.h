#pragma once

#include "controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_timeout_handler
{
    class RobotCore;
    class Robot;

	///Cartesian controller makes robot try to approach given position with given stiffness and daming matrix
	class CartesianController : public Controller
	{
    friend RobotCore;
    private:
        //Output
        Eigen::Matrix<double, 3, 1> _target_position;
        Eigen::Quaterniond _target_orientation;
        Eigen::Matrix<double, 3, 3> _translation_stiffness;
        Eigen::Matrix<double, 3, 3> _rotation_stiffness;
        Eigen::Matrix<double, 3, 3> _translation_damping;
        Eigen::Matrix<double, 3, 3> _rotation_damping;
        bool _control_rotation;

        //Buffer for laties
        Eigen::Matrix<double, 3, 1> _late_target_position;
        bool _late_update_target_position       = false;
        Eigen::Quaterniond _late_target_orientation;
        bool _late_update_target_orientation    = false;
		Eigen::Matrix<double, 3, 3> _late_translation_stiffness;
        bool _late_update_translation_stiffness = false;
        Eigen::Matrix<double, 3, 3> _late_rotation_stiffness;
        bool _late_update_rotation_stiffness    = false;
		Eigen::Matrix<double, 3, 3> _late_translation_damping;
        bool _late_update_translation_damping   = false;
        Eigen::Matrix<double, 3, 3> _late_rotation_damping;
        bool _late_update_rotation_damping      = false;
        bool _late_control_rotation;
        bool _late_update_control_rotation      = false;

        //Overloadings
        virtual void _robot_output_to_output();
        virtual void _calculate_result();
        virtual void _robot_output_to_late_output();
        virtual void _late_output_to_output();
        virtual ControllerType typ() const;
		using Controller::Controller;
	};
}