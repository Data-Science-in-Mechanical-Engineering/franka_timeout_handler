#pragma once

#include "controller.h"
#include <franka/robot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <thread>

namespace franka_real_time
{
    class Robot;

	///Cartesian controller makes robot try to approach given position with given stiffness and daming matrix
	class CartesianController : public Controller
	{
    friend Robot;
    private:
        enum class ReceiveState
        {
            other,
            receive,
            post_receive,
            receive_and_send,
            destructor
        };

        enum class SendState
        {
            other,
            pre_wait,
            wait,
            post_wait
        };

        //States
        ReceiveState _receive_state             = ReceiveState::other;
        SendState _send_state                   = SendState::other;
        unsigned int _frequency_divider_count   = 0;
        Robot *_robot                           = nullptr;
        
        //Calculation
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;
        Eigen::Affine3d _transform;
        Eigen::Matrix<double, 3, 1> _position;
        Eigen::Quaterniond _orientation;
        Eigen::Matrix<double, 6, 1> _velocity_rotation;
        Eigen::Matrix<double, 7, 1> _coriolis;
        Eigen::Matrix<double, 6, 7> _jacobian;
        Eigen::Matrix<double, 7, 7> _rotation_correction;

        //Current values
        unsigned int _timeout;
        Eigen::Matrix<double, 3, 1> _target_position;
        Eigen::Quaterniond _target_orientation;
        Eigen::Matrix<double, 3, 3> _translation_stiffness;
        Eigen::Matrix<double, 3, 3> _rotation_stiffness;
        Eigen::Matrix<double, 3, 3> _translation_damping;
        Eigen::Matrix<double, 3, 3> _rotation_damping;
        bool _control_rotation;
        double _joint_torques_limit;
        unsigned int _frequency_divider;
        
        Eigen::Matrix<double, 7, 1> _joint_torques;
        bool _joint_torques_finished;

        //Buffer for laties
        unsigned int _late_timeout;
        bool _late_update_timeout               = false;
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
        bool _late_joint_torques_limit;
        bool _late_update_joint_torques_limit   = false;
        bool _late_frequency_divider;
        bool _late_update_frequency_divider     = false;

        Eigen::Matrix<double, 7, 1> _late_joint_torques;

        //Technical
        pthread_cond_t _receive_condition   = PTHREAD_COND_INITIALIZER;
        pthread_cond_t _send_condition      = PTHREAD_COND_INITIALIZER;
        pthread_mutex_t _mutex              = PTHREAD_MUTEX_INITIALIZER;
        pthread_t _backend_thread;

        void _state_to_input(const franka::RobotState &robot_state);
        void _input_to_robot_input();
        void _calculate_temporary(const franka::RobotState &robot_state);
        void _robot_output_to_output();
        void _calculate_result();
        void _result_to_robot_result();

        void _robot_output_to_late_output();
        void _late_output_to_output();
        void _result_to_late_result();
        void _late_result_to_robot_result();

        void _control(const franka::RobotState &robot_state);
        Eigen::Matrix<double, 7, 7> _pseudoinverse(const Eigen::Matrix<double, 7, 7> &a, double epsilon);

		CartesianController(Robot *robot);
        virtual void receive();
		virtual void send();
		virtual void receive_and_send();
		virtual void send_and_receive();
        ~CartesianController();
	};
}
