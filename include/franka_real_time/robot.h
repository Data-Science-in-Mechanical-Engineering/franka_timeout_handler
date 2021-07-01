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

        //Input
        Eigen::Matrix<double, 7, 1> _joint_positions;
        Eigen::Matrix<double, 7, 1> _joint_velocities;
		Eigen::Matrix<double, 3, 1> _position;
        Eigen::Quaterniond _orientation;
		Eigen::Matrix<double, 3, 1> _velocity;
        Eigen::Matrix<double, 3, 1> _rotation;

        //Output
        unsigned int _timeout;
        Eigen::Matrix<double, 3, 1> _target_position;
        Eigen::Quaterniond _target_orientation;
        Eigen::Matrix<double, 3, 3> _translation_stiffness;
        Eigen::Matrix<double, 3, 3> _rotation_stiffness;
        Eigen::Matrix<double, 3, 3> _translation_damping;
        Eigen::Matrix<double, 3, 3> _rotation_damping;
        bool _control_rotation;

		Update _update_timeout               = Update::yes;
        Update _update_target_position       = Update::yes;
        Update _update_target_orientation    = Update::yes;
        Update _update_translation_stiffness = Update::yes;
        Update _update_rotation_stiffness    = Update::yes;
        Update _update_translation_damping   = Update::yes;
        Update _update_rotation_damping      = Update::yes;
        Update _update_control_rotation      = Update::yes;

        //Result
		Eigen::Matrix<double, 7, 1> _joint_torques;
        Update _update_joint_torques         = Update::yes;
		bool _late                           = false;

	public:
        ///Returns joint positions (input)
		Eigen::Matrix<double, 7, 1> get_joint_positions()   const;
        ///Returns joint velocities (input)
		Eigen::Matrix<double, 7, 1> get_joint_velocities()  const;
		///Returns cartesian position (input)
		Eigen::Matrix<double, 3, 1> get_position()          const;
        ///Returns cartesian orientation (input)
		Eigen::Quaterniond get_orientation()                const;
        ///Returns cartesian orientation in Euler angles: roll, pitch, yaw (input)
        Eigen::Matrix<double, 3, 1> get_orientation_euler() const;
		///Returns cartesian velocity (input)
		Eigen::Matrix<double, 3, 1> get_velocity()          const;
        ///Returns cartesian rotation (input)
		Eigen::Matrix<double, 3, 1> get_rotation()          const;

        ///Sets timeout in microsencods (output)
        void set_timeout(unsigned int timeout);
        ///Sets cartesian position of tartget (output)
        void set_target_position(Eigen::Matrix<double, 3, 1> position);
        ///Sets cartesian orientation of tartget (output)
        void set_target_orientation(Eigen::Quaterniond orientation);
        ///Sets cartesian orientation of target in Euler angles: roll, pitch, yaw (output)
        void set_target_orientation_euler(Eigen::Matrix<double, 3, 1> euler);
        ///Sets translation stiffness matrix (output)
        void set_translation_stiffness(Eigen::Matrix<double, 3, 3> stiffness);
        ///Sets rotation stiffness matrix (output)
        void set_rotation_stiffness(Eigen::Matrix<double, 3, 3> stiffness);
        ///Sets translation damping matrix (output)
        void set_translation_damping(Eigen::Matrix<double, 3, 3> damping);
        ///Sets translation damping matrix (output)
        void set_rotation_damping(Eigen::Matrix<double, 3, 3> damping);
        ///Sets indicator if rotation should be handled (output)
        void set_control_rotation(bool control);
        ///Returns timeout in microsencods (output)
        unsigned int get_timeout()                                  const;
        ///Returns cartesian position of tartget (output)
        Eigen::Matrix<double, 3, 1> get_target_position()           const;
        ///Returns cartesian orientation of tartget (output)
        Eigen::Quaterniond get_target_orientation()                 const;
        ///Returns cartesian orientation of target in Euler angles: roll, pitch, yaw (output)
        Eigen::Matrix<double, 3, 1> get_target_orientation_euler()  const;
        ///Returns translation stiffness matrix (output)
        Eigen::Matrix<double, 3, 3> get_translation_stiffness()     const;
        ///Returns rotation stiffness matrix (output)
        Eigen::Matrix<double, 3, 3> get_rotation_stiffness()        const;
        ///Returns translation damping matrix (output)
        Eigen::Matrix<double, 3, 3> get_translation_damping()       const;
        ///Returns translation damping matrix (output)
        Eigen::Matrix<double, 3, 3> get_rotation_damping()          const;
        ///Returns indicator if rotation should be handled (output)
        bool get_control_rotation()                                 const;
        ///Sets update mode of timeout (output)
        void set_timeout_update(Update update);
        ///Sets update mode of target cartesian position (output)
        void set_target_position_update(Update update);
        ///Sets update mode of target cartesian orientation (output)
        void set_target_orientation_update(Update update);
        ///Sets update mode of translation stiffness matrix (output)
        void set_translation_stiffness_update(Update update);
        ///Sets update mode of rotation stiffness matrix (output)
        void set_rotation_stiffness_update(Update update);
        ///Sets update mode of translation damping matrix (output)
        void set_translation_damping_update(Update update);
        ///Sets update mode of translation damping matrix (output)
        void set_rotation_damping_update(Update update);
        ///Sets update mode of indicator if rotation should be handled (output)
        void set_control_rotation_update(Update update);
        ///Returns update mode of timeout (output)
        Update get_timeout_update()                 const;
        ///Returns update mode of target cartesian position (output)
        Update get_target_position_update()         const;
        ///Returns update mode of target cartesian orientation (output)
        Update get_target_orientation_update()      const;
        ///Returns update mode of translation stiffness matrix (output)
        Update get_translation_stiffness_update()   const;
        ///Returns update mode of rotation stiffness matrix (output)
        Update get_rotation_stiffness_update()      const;
        ///Returns update mode of translation damping matrix (output)
        Update get_translation_damping_update()     const;
        ///Returns update mode of translation damping matrix (output)
        Update get_rotation_damping_update()        const;
        ///Returns update mode of indicator if rotation should be handled (output)
        Update get_control_rotation_update()        const;

		///Returns torques sent to the robot (result)
        Eigen::Matrix<double, 7, 1> get_joint_torques() const;
        ///Returns if `send()` was called too late (result)
		bool get_late() const;
        ///Sets update mode of torques (result)
        void set_joint_torques_update(Update update);
        ///Returns update mode of torques (result)
        Update get_joint_torques_update() const;

        ///Sets update mode to all output and result variables
		void set_update(Update update);
        
        ///@param ip IPv4 address of the robot
		Robot(std::string ip);
        ///Starts cartesian controller
        void control_cartesian();
		///Waits for next signal (if controller is running) and refreshes inputs
		void receive();
		///Sends signal back, updates outputs, refreshes results
		void send();
		///Waits for signal and immediately sends signal back with no chance to be late, refreshes inputs and results, updates outputs
		void receive_and_send();
		///Sends signal back and waits for new signal, updates outputs, refreshes results and then inputs
		void send_and_receive();
        ///Returns norm of distance between position and target
        double distance() const;
        ///Iterates till the robot reaches target with given tolerance or till time expire
        ///@param tolerance Norm of distance between position and target that can be tolerated
        ///@param timeout Maximal time in milliseconds (number of iterations)
        void loop(double tolerance, unsigned int timeout);
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