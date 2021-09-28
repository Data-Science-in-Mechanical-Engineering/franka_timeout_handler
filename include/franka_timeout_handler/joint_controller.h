#pragma once

#include "controller.h"
#include <franka/robot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <thread>
#include <cstdint>

namespace franka_timeout_handler
{
    class RobotCore;
    class Robot;
    
	///Joint controller makes robot try to approach given joint-space position with given stiffness and daming matrix
	class JointController : public Controller
	{
    friend RobotCore;
    private:
        //Output
        Eigen::Matrix<double, 7, 1> _target_joint_positions;
        Eigen::Matrix<double, 7, 7> _joint_stiffness;
        Eigen::Matrix<double, 7, 7> _joint_damping;

        //Buffer for laties
        Eigen::Matrix<double, 7, 1> _late_target_joint_positions;
        bool _late_update_target_joint_positions    = false;
        Eigen::Matrix<double, 7, 7> _late_joint_stiffness;
        bool _late_update_joint_stiffness           = false;
        Eigen::Matrix<double, 7, 7> _late_joint_damping;
        bool _late_update_joint_damping             = false;

        //Overloadings
        virtual void _robot_output_to_output();
        virtual void _calculate_result();
        virtual void _robot_output_to_late_output();
        virtual void _late_output_to_output();
        virtual ControllerType typ() const;
        using Controller::Controller;
	};
}