#pragma once

#include "controller.h"
#include <franka/robot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <thread>

namespace franka_real_time
{
    class Robot;

	///Cartesial controller makes robot try to approach given position with given stiffness and daming matrix
	class CartesialController : public Controller
	{
    private:
        Robot *_robot           = nullptr;

        //States
        bool _front_receiving   = false;
        bool _front_received    = false;

        bool _back_receiving    = false; //Set by back, indicates wait() was called
        bool _back_received     = false; //Set by front, indicates front has arrived
        bool _back_timeout      = false; //Set by back, indicated front has not arrived

        bool _finish            = false;
        
        //Calculation
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;
        Eigen::Affine3d _transform;
        Eigen::Matrix<double, 3, 1> _position;
        Eigen::Quaterniond _orientation;
        Eigen::Matrix<double, 6, 1> _velocity_rotation;
        Eigen::Matrix<double, 7, 1> _coriolis;
        Eigen::Matrix<double, 6, 7> _jacobian;

        //Current values
        unsigned int _timeout;
        Eigen::Matrix<double, 3, 1> _target_position;
        Eigen::Quaterniond _target_orientation;
        Eigen::Matrix<double, 3, 3> _translation_stiffness;
        Eigen::Matrix<double, 3, 3> _rotation_stiffness;
        Eigen::Matrix<double, 3, 3> _translation_damping;
        Eigen::Matrix<double, 3, 3> _rotation_damping;
        bool _control_rotation;
        
        Eigen::Matrix<double, 7, 1> _joint_torques;

        //Buffer for laties
        unsigned int _late_timeout;
        bool _late_timeout_update               = false;
		Eigen::Matrix<double, 3, 1> _late_target_position;
        bool _late_target_position_update       = false;
		Eigen::Quaterniond _late_target_orientation;
        bool _late_target_orientation_update    = false;
		Eigen::Matrix<double, 3, 3> _late_translation_stiffness;
        bool _late_translation_stiffness_update = false;
        Eigen::Matrix<double, 3, 3> _late_rotation_stiffness;
        bool _late_rotation_stiffness_update    = false;
		Eigen::Matrix<double, 3, 3> _late_translation_damping;
        bool _late_translation_damping_update   = false;
        Eigen::Matrix<double, 3, 3> _late_rotation_damping;
        bool _late_rotation_damping_update      = false;
        bool _late_control_rotation;
        bool _late_control_rotation_update      = false;


        Eigen::Matrix<double, 7, 1> _late_joint_torques;

        //Technical
        pthread_cond_t _receive_condition   = PTHREAD_COND_INITIALIZER;
        pthread_cond_t _send_condition      = PTHREAD_COND_INITIALIZER;
        pthread_mutex_t _mutex              = PTHREAD_MUTEX_INITIALIZER;
        std::thread _backend_thread;

        void _calculate_joint_torques();
        void _control(const franka::RobotState &robot_state, franka::Torques *joint_torques_array);
        static void _control_thread_function(CartesialController *controller);

    public:
		CartesialController(Robot *robot);
        virtual void receive();
		virtual void send();
		virtual void receive_and_send();
		virtual void send_and_receive();
        ~CartesialController();
	};
}