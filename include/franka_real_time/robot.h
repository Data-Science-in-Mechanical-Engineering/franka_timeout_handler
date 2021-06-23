#pragma once

#include "update.h"
#include "cartesian_controller.h"
#include <thread>
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_real_time
{
    class CartesianController;

	///Franka Panda robot@n
    ///Sends and receies signals from the robot. The main freature of the API is that the user may try to update "output" fields between `send()` and `receive()`,
    ///so that new values are applied on the save tact of the robot@n
    ///Robot's fields are divided in three groups:@n
    /// - Input: fields are refreshed when signal is received
    /// - Output: fields are send to robot when signal is sent
    /// - Result: fileds are refreshed when signal is sent
	class Robot
	{
    friend CartesianController;
    private:
        franka::Robot _robot;
        franka::Model *_model = nullptr;
        Controller *_controller = nullptr;

	public:
        ///Joint positions, belongs to "input" group
		Eigen::Matrix<double, 7, 1> joint_positions;
        ///Joint velocities, belongs to "input" group
		Eigen::Matrix<double, 7, 1> joint_velocities;
		///Cartesian position, belongs to "input" group
		Eigen::Matrix<double, 3, 1> position;
        ///Cartesian orientation, belongs to "input" group
		Eigen::Quaterniond orientation;
		///Cartesian velocity, belongs to "input" group
		Eigen::Matrix<double, 3, 1> velocity;
        ///Cartesian rotation, belongs to "input" group
		Eigen::Matrix<double, 3, 1> rotation;

        ///Time in microseconds the controller waits for `send()` after `receive()`, belongs to "output" group
		unsigned int timeout;
        ///Cartesian position of tartget, belongs to "output" group
		Eigen::Matrix<double, 3, 1> target_position;
        ///Cartesian orientation of tartget, belongs to "output" group
		Eigen::Quaterniond target_orientation;
        ///Translation stiffness matrix, belongs to "output" group
		Eigen::Matrix<double, 3, 3> translation_stiffness;
        ///Rotation stiffness matrix, belongs to "output" group
        Eigen::Matrix<double, 3, 3> rotation_stiffness;
        ///Translation damping matrix, belongs to "output" group
		Eigen::Matrix<double, 3, 3> translation_damping;
        ///Translation damping matrix, belongs to "output" group
        Eigen::Matrix<double, 3, 3> rotation_damping;
        ///Indicator if rotation should be handled, belongs to "output" group
        bool control_rotation;

        ///Application mode of `timeout` field
		Update timeout_update               = Update::if_not_late;
        ///Application mode of `cartesian_target_position`
		Update target_position_update       = Update::if_not_late;
        ///Application mode of `cartesian_target_orientation`
		Update target_orientation_update    = Update::if_not_late;
        ///Application mode of `translation_stiffness`
		Update translation_stiffness_update = Update::if_not_late;
        ///Application mode of `rotation_stiffness`
		Update rotation_stiffness_update    = Update::if_not_late;
        ///Application mode of `translation_damping`
		Update translation_damping_update   = Update::if_not_late;
        ///Application mode of `rotation_damping`
		Update rotation_damping_update      = Update::if_not_late;
        ///Application mode of `control_rotation`
        Update control_rotation_update      = Update::if_not_late;

		//Result
        ///Torques sent to the robot, belongs to "result" group
		Eigen::Matrix<double, 7, 1> joint_torques;
        ///Application mode of `damping`
        Update joint_torques_update         = Update::if_not_late;
        ///Indicator if `send()` was called too late, belongs to "result" group
		bool late                           = false;

        ///Creates robot
		Robot(std::string ip);
        ///Starts cartesian controller
        void control_cartesian();
        ///Sets update mode to all outputs
		void update(Update update);
		///Waits for next signal and refreshes inputs
		void receive();
		///Sends signal back, updates outputs, refreshes results
		void send();
		///Waits for signal and immediately sends signal back with no chance to be late, refreshes inputs and results, updates outputs
		void receive_and_send();
		///Sends signal back and waits for new signal, updates outputs, refreshes results and then inputs
		void send_and_receive();
        ///Returns norm of distance between position and target
        double distance() const;
        ///Stops controller
        void stop();
		///Destroys robot
		~Robot();
	};
}