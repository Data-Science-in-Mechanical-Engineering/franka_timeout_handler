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

	///Franka Panda robot
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
        ///@param ip IPv4 address of the robot
		Robot(std::string ip);
        ///Starts cartesian controller
        void control_cartesian();
        ///Sets update mode to all outputs
        ///@param update Update mode to be set
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

/** @mainpage Welcome to Franka real-time
Here you will find a small and simple library that allows non-real-time programs to operate with Franka Panda robot at hight frequency.

@tableofcontents

@section Usage
In order to understand what this library does, it may be helpful to take a look at an example of robot control with [libfranka](https://github.com/frankaemika/libfranka):
@code {.cpp}
franka::Robot robot("192.168.0.1");
robot.control([](const franka::RobotState &robot_state, franka::Duration time) -> franka::Torques
{
	//Robot called our function, we have have ~300 microseconds to analyze `robot_state` and return torques
	return calculate_torques_from_robot_state(robot_state);
});
@endcode

*for `Python` programmers: here a multi-line lambda function that accepts state and returns torques, is passed `robot.control` function*

Direct `Python` wrapper of this code is possible, but if `Python` freezes (for example because of garbage collector), an error would occur. The library's goal is to avoid this situation and transform the code above into this:
@code{.cpp}
franka_real_time::Robot robot("192.168.0.1");
robot.start_cartesian();
//We started control loop, now we need to wait
robot.receive();
//The robot called internal function, we have ~300 microseconds to analyze and change `robot`
robot.send();
//We might have exceeded the timeout and our changes might be impossible to apply, but nothing critical happened to the robot
printf(robot.late ? "Late" : "Not late");
@endcode

Fields of `Robot` class can be divided in three groups:
 - Input: fields are refreshed with `receive()` (positions, velocities, etc.)
 - Output: fields are applied with `send()` (stiffness, damping, etc.)
 - Result: fileds are refreshed with `send()` (torques and lateness indicator)

Currently there exists only one controller: `CartesianController`, which makes the robot stand in given cartesian position.

@section Dependencies
The library depends on:
 - [libfranka](https://github.com/frankaemika/libfranka) (as part of ROS)
 - [Eigen](https://eigen.tuxfamily.org)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [CMake](https://cmake.org) >= `3.10.2`
 - Fully preemptable Linux kernel
 - `C++11` compatible compiler

@section Installation
@code
mkdir build
cd build
cmake ..
cmake --build .
@endcode

@section Documentation
[Doxygen](https://www.doxygen.nl) documentation is provided.
*/