#pragma once

#include <franka/robot.h>
#include <Eigen/Dense>
#include <thread>
#include <cstdint>

namespace franka_real_time
{
    class RobotCore;
    class Robot;

    ///Abstract controller
    class Controller
    {
    friend RobotCore;
    friend Robot;
    protected:
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
        
        //Input
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;
        Eigen::Affine3d _transform;
        Eigen::Matrix<double, 3, 1> _position;
        Eigen::Quaterniond _orientation;
        Eigen::Matrix<double, 6, 1> _velocity_rotation;
        std::uint64_t _call;

        //Calculation
        Eigen::Matrix<double, 7, 1> _coriolis;
        Eigen::Matrix<double, 6, 7> _jacobian;
        Eigen::Matrix<double, 7, 1> _joint_torques_maximum;

        //Output
        unsigned int _timeout;
        double _joint_torques_limit;
        unsigned int _frequency_divider;

        //Result
        Eigen::Matrix<double, 7, 1> _joint_torques;
        bool _joint_torques_finished;

        //Buffer for laties
        unsigned int _late_timeout;
        bool _late_update_timeout               = false;
		double _late_joint_torques_limit;
        bool _late_update_joint_torques_limit   = false;
        unsigned int _late_frequency_divider;
        bool _late_update_frequency_divider     = false;
        Eigen::Matrix<double, 7, 1> _late_joint_torques;

        //Technical
        RobotCore *_robot                   = nullptr;
        pthread_cond_t _receive_condition   = PTHREAD_COND_INITIALIZER;
        pthread_cond_t _send_condition      = PTHREAD_COND_INITIALIZER;
        pthread_mutex_t _mutex              = PTHREAD_MUTEX_INITIALIZER;
        pthread_t _backend_thread;

        //Helper functions, can be overloaded to add new fileds
        virtual void _state_to_input(const franka::RobotState &robot_state);
        virtual void _input_to_robot_input();
        virtual void _calculate_temporary(const franka::RobotState &robot_state);
        virtual void _robot_output_to_output();
        virtual void _calculate_result() = 0;
        virtual void _result_to_robot_result();

        virtual void _robot_output_to_late_output();
        virtual void _late_output_to_output();
        virtual void _result_to_late_result();
        virtual void _late_result_to_robot_result();

        virtual bool typ() = 0;

        //Main functionality, can not be overloaded
        void _control(const franka::RobotState &robot_state);
        void receive();
		void send();
		void receive_and_send();
		void send_and_receive();
        Controller(RobotCore *robot_core);
        ~Controller();
        static void set_default(RobotCore *robot_core);
    };
}