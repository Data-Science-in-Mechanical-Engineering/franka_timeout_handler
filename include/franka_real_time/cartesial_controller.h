#pragma once

#include "controller.h"
#include "update.h"
#include "robot.h"
#include <thread>
#include <pthread.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_real_time
{
	///Cartesial controller makes robot try to approach given position with given stiffness and daming matrix
	class CartesialController : public Controller
	{
    private:
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
        Eigen::Matrix<double, 7, 1> _coriolis;
        Eigen::Matrix<double, 6, 7> _jacobian;
        Eigen::Matrix<double, 6, 1> _velocity;

        //Current values
        unsigned int _timeout;
        Eigen::Matrix<double, 3, 1> _target_position;
        Eigen::Quaterniond _target_orientation;
        Eigen::Matrix<double, 6, 6> _stiffness;
        Eigen::Matrix<double, 6, 6> _damping;
        
        Eigen::Matrix<double, 7, 1> _joint_torques;

        //Buffer for laties
        unsigned int _late_timeout;
        bool _late_timeout_update           = false;
		Eigen::Matrix<double, 3, 1> _late_target_position;
        bool _late_target_position_update   = false;
		Eigen::Quaterniond _late_target_orientation;
        bool _late_target_orientation_update= false;
		Eigen::Matrix<double, 6, 6> _late_stiffness;
        bool _late_stiffness_update         = false;
		Eigen::Matrix<double, 6, 6> _late_damping;
        bool _late_damping_update           = false;

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
        ///Joint positions, belongs to "input" group, is refreshed when signal is received
		Eigen::Matrix<double, 7, 1> joint_positions;
        ///Joint velocities, belongs to "input" group, is refreshed when signal is received
		Eigen::Matrix<double, 7, 1> joint_velocities;
		///Cartesial position, belongs to "input" group, is refreshed when signal is received
		Eigen::Matrix<double, 3, 1> position;
        ///Cartesial orientation, belongs to "input" group, is refreshed when signal is received
		Eigen::Quaterniond orientation;
		
        ///Time in microseconds the controller will wait for `send()` after `receive()`, belongs to "output" group, may be applied when signal is sent
		unsigned int timeout;
        ///Cartesial position of tartget, belongs to "output" group, may be applied when signal is sent
		Eigen::Matrix<double, 3, 1> target_position;
        ///Cartesial orientation of tartget, belongs to "output" group, may be applied when signal is sent
		Eigen::Quaterniond target_orientation;
        ///Stiffness matrix, belongs to "output" group, may be applied when signal is sent
		Eigen::Matrix<double, 6, 6> stiffness;
        ///Stiffness matrix, belongs to "output" group, may be applied when signal is sent
		Eigen::Matrix<double, 6, 6> damping;
        ///Application mode of `timeout` field
		Update timeout_update               = Update::if_not_late;
        ///Application mode of `cartesial_target_position`
		Update target_position_update       = Update::if_not_late;;
        ///Application mode of `cartesial_target_orientation`
		Update target_orientation_update    = Update::if_not_late;;
        ///Application mode of `stiffness`
		Update stiffness_update             = Update::if_not_late;;
        ///Application mode of `damping`
		Update damping_update               = Update::if_not_late;;
		
		//Result
        ///Torques sent to the robot, belongs to "result" group, is refreshed when signal is sent
		Eigen::Matrix<double, 7, 1> joint_torques;
        ///Application mode of `damping`
        Update joint_torques_update         = Update::if_not_late;;
        ///Indicator if `send()` was called too late, belongs to "result" group, is refreshed when signal is sent
		bool late                           = false;
		
		///Creates cartesial controller
		CartesialController(Robot &robot);
		///Sets update mode to all outputs
		void update(Update update);
		///Waits for next signal and refreshes inputs
		void receive();
		///Sends signal back, updates outputs, refreshes results
		void send();
		///Waits for signal and immediately sends signal back, refreshes inputs and results, updates outputs
		void receive_and_send();
		///Sends signal back and waits for new signal, updates outputs, refreshes results and then inputs
		void send_and_receive();
        ///Destroys cartesial controller
        ~CartesialController();
	};
}