#pragma once

#include "update.h"
#include "controller.h"
#include "controller_type.h"
#include "constants.h"
#include <thread>
#include <cstdint>
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace franka_timeout_handler
{
    class Controller;
    class CartesianController;
    class JointController;

    ///Basic functionality of Franka Emika robot
    class RobotCore
    {
    friend Controller;
    friend CartesianController;
    friend JointController;
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
        std::uint64_t _call;

        //Output
        unsigned int _timeout                               = default_timeout;
        double _joint_torques_limit                         = default_torques_limit;
        unsigned int _frequency_divider                     = default_frequency_divider;
        Eigen::Matrix<double, 3, 1> _target_position;       //Read from robot by default
        Eigen::Quaterniond _target_orientation;             //Read from robot by default
        Eigen::Matrix<double, 3, 3> _translation_stiffness  = default_translation_stiffness;
        Eigen::Matrix<double, 3, 3> _rotation_stiffness     = default_rotation_stiffness;
        Eigen::Matrix<double, 3, 3> _translation_damping    = default_translation_damping;
        Eigen::Matrix<double, 3, 3> _rotation_damping       = default_rotation_damping;
        bool _control_rotation                              = default_control_rotation;
        Eigen::Matrix<double, 7, 1> _target_joint_positions;//Read from robot by default
        Eigen::Matrix<double, 7, 7> _joint_stiffness        = default_joint_stiffness;
        Eigen::Matrix<double, 7, 7> _joint_damping          = default_joint_damping;

        Update _update_timeout                  = default_update;
        Update _update_joint_torques_limit      = default_update;
        Update _update_frequency_divider        = default_update;
        Update _update_target_position          = default_update;
        Update _update_target_orientation       = default_update;
        Update _update_translation_stiffness    = default_update;
        Update _update_rotation_stiffness       = default_update;
        Update _update_translation_damping      = default_update;
        Update _update_rotation_damping         = default_update;
        Update _update_control_rotation         = default_update;
        Update _update_target_joint_positions   = default_update;
        Update _update_joint_stiffness          = default_update;
        Update _update_joint_damping            = default_update;
        
        //Result
        Eigen::Matrix<double, 7, 1> _joint_torques  = Eigen::Matrix<double, 7, 1>::Zero();
        Update _update_joint_torques                = default_update;
        bool _late                                  = false;

    public:
        //Basic:
        ///Creates robot core
        ///@param ip IPv4 address of the robot
        RobotCore(std::string ip);
        ///Starts controller
        ///@param typ Type of controller
        void start(ControllerType typ);
        ///Stops controller
        void stop();
        ///Returns if controller is started
        bool started() const;
        ///Returns type of controller
        ControllerType typ() const;
        ///Waits for next signal (if controller is running) and refreshes inputs
        void receive();
        ///Sends signal back, updates outputs, refreshes results
        void send();
        ///Waits for signal and immediately sends signal back with no chance to be late, refreshes inputs and results, updates outputs
        void receive_and_send();
        ///Sends signal back and waits for new signal, updates outputs, refreshes results and then inputs
        void send_and_receive();       

        //Input:
        ///Returns joint positions (input)
        Eigen::Matrix<double, 7, 1> get_joint_positions()   const;
        ///Returns joint velocities (input)
        Eigen::Matrix<double, 7, 1> get_joint_velocities()  const;
        ///Returns cartesian position (input)
        Eigen::Matrix<double, 3, 1> get_position()          const;
        ///Returns cartesian orientation (input)
        Eigen::Quaterniond get_orientation()                const;
        ///Returns cartesian orientation in Euler angles: yaw, pitch, roll (input)
        Eigen::Matrix<double, 3, 1> get_orientation_euler() const;
        ///Returns cartesian orientation as quanterion, but in XYZW form (input)
        Eigen::Matrix<double, 4, 1> get_orientation_wxyz()  const;
        ///Returns cartesian velocity (input)
        Eigen::Matrix<double, 3, 1> get_velocity()          const;
        ///Returns cartesian rotation (input)
        Eigen::Matrix<double, 3, 1> get_rotation()          const;
        ///Returns call number, call happens every millisecond (input)
        std::uint64_t get_call()                            const;

        //Output:
        ///Sets timeout in microsencods (output)
        void set_timeout(unsigned int timeout);
        ///Sets security limit for torques, 1.0 to full torques (output)
        void set_joint_torques_limit(double limit);
        ///Sets frequency divider (output)
        void set_frequency_divider(unsigned int divider);
        ///Sets cartesian position of target (output)
        void set_target_position(const Eigen::Matrix<double, 3, 1> &position);
        ///Sets cartesian orientation of tartget (output)
        void set_target_orientation(const Eigen::Quaterniond &orientation);
        ///Sets cartesian orientation of target in Euler angles: yaw, pitch, roll (output)
        void set_target_orientation_euler(const Eigen::Matrix<double, 3, 1> &euler);
        ///Sets cartesian orientation of target as quanterion, but in XYZW form (output)
        void set_target_orientation_wxyz(const Eigen::Matrix<double, 4, 1> &wxyz);
        ///Sets translation stiffness matrix (output)
        void set_translation_stiffness(const Eigen::Matrix<double, 3, 3> &stiffness);
        ///Sets rotation stiffness matrix (output)
        void set_rotation_stiffness(const Eigen::Matrix<double, 3, 3> &stiffness);
        ///Sets translation damping matrix (output)
        void set_translation_damping(const Eigen::Matrix<double, 3, 3> &damping);
        ///Sets rotation damping matrix (output)
        void set_rotation_damping(const Eigen::Matrix<double, 3, 3> &damping);
        ///Sets indicator if orientation should be controller (output)
        void set_control_rotation(bool control);
        ///Sets joint-space target (output)
        void set_target_joint_positions(const Eigen::Matrix<double, 7, 1> &positions);
        ///Sets joint-space stiffness matrix (output)
        void set_joint_stiffness(const Eigen::Matrix<double, 7, 7> &stiffness);
        ///Sets joint-space damping matrix (output)
        void set_joint_damping(const Eigen::Matrix<double, 7, 7> &damping);
        ///Returns timeout in microsencods (output)
        unsigned int get_timeout()                                  const;
        ///Returns security limit for torques (output)
        double get_joint_torques_limit()                            const;
        ///Returns frequency divider (output)
        unsigned int get_frequency_divider()                        const;
        ///Returns cartesian position of tartget (output)
        Eigen::Matrix<double, 3, 1> get_target_position()           const;
        ///Returns cartesian orientation of tartget (output)
        Eigen::Quaterniond get_target_orientation()                 const;
        ///Returns cartesian orientation of target in Euler angles: yaw, pitch, roll (output)
        Eigen::Matrix<double, 3, 1> get_target_orientation_euler()  const;
        ///Returns cartesian orientation of target as quanterion, but in XYZW form (output)
        Eigen::Matrix<double, 4, 1> get_target_orientation_wxyz()   const;
        ///Returns translation stiffness matrix (output)
        Eigen::Matrix<double, 3, 3> get_translation_stiffness()     const;
        ///Returns rotation stiffness matrix (output)
        Eigen::Matrix<double, 3, 3> get_rotation_stiffness()        const;
        ///Returns translation damping matrix (output)
        Eigen::Matrix<double, 3, 3> get_translation_damping()       const;
        ///Returns rotation damping matrix (output)
        Eigen::Matrix<double, 3, 3> get_rotation_damping()          const;
        ///Returns if orientation should be controlled (output)
        bool get_control_rotation()                                 const;
        ///Returns joint-space target (output)
        Eigen::Matrix<double, 7, 1> get_target_joint_positions()    const;
        ///Returns joint-space stiffness matrix (output)
        Eigen::Matrix<double, 7, 7> get_joint_stiffness()           const;
        ///Returns joint-space damping matrix (output)
        Eigen::Matrix<double, 7, 7> get_joint_damping()             const;
        ///Sets update mode of timeout (output)
        void set_timeout_update(Update update);
        ///Sets update mode of security limit for torques (output)
        void set_joint_torques_limit_update(Update update);
        ///Sets update mode of frequency divider (output)
        void set_frequency_divider_update(Update update);
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
        ///Sets update mode of rotation damping matrix (output)
        void set_rotation_damping_update(Update update);
        ///Sets update mode of indicator if orientation should be controlled (output)
        void set_control_rotation_update(Update update);
        ///Sets update mode of joint-space target (output)
        void set_target_joint_positions_update(Update update);
        ///Sets update mode of joint-space stiffness matrix (output)
        void set_joint_stiffness_update(Update update);
        ///Sets update mode of joint-space damping matrix (output)
        void set_joint_damping_update(Update update);
        ///Returns update mode of timeout (output)
        Update get_timeout_update()                 const;
        ///Returns update mode of security limit for torques (output)
        Update get_joint_torques_limit_update()     const;
        ///Returns update mode of frequency divider (output)
        Update get_frequency_divider_update()       const;
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
        ///Returns update mode of rotation damping matrix (output)
        Update get_rotation_damping_update()        const;
        ///Returns update mode of indicator if orientation should be controlled (output)
        Update get_control_rotation_update()        const;
        ///Returns update mode of joint-space target (output)
        Update get_target_joint_positions_update()  const;
        ///Returns update mode of joint-space stiffness matrix (output)
        Update get_joint_stiffness_update()         const;
        ///Returns update mode of joint-space damping matrix (output)
        Update get_joint_damping_update()           const;

        //Result:
        ///Returns torques sent to the robot (result)
        Eigen::Matrix<double, 7, 1> get_joint_torques() const;
        ///Returns if `send()` was called too late (result)
        bool get_late() const;
        ///Sets update mode of torques (result)
        void set_joint_torques_update(Update update);
        ///Returns update mode of torques (result)
        Update get_joint_torques_update() const;

        ///Destroys robot
        ~RobotCore();
    };
}

/** @mainpage Welcome to `franka_timeout_handler 1.0.0`!
# Welcome to `franka_timeout_handler 1.0.0`
here you will find a library for Franka Emika Panda robot, which allows you to send messages to real-time controller from non-real-time application without breaking robot's real-time requirements.

@tableofcontents

@section Usage
In order to understand what this library does, it may be helpful to take a look at an example of robot control with [libfranka](https://github.com/frankaemika/libfranka):
```{.cpp}
franka::Torques control(const franka::RobotState &robot_state, franka::Duration time)
{
	//Robot called the function, now it has ~300 microseconds to analyze `robot_state` and return torques
	return calculate_torques_from_robot_state(robot_state);
}

int main()
{
	//Create robot and pass control function to it
	franka::Robot robot("192.168.0.1");
	robot.control(control);
}
```

Straightforward `Python` translation of this code is possible, but if `Python` freezes (for example because of the garbage collector), an error will occur. The library's goal is to avoid this situation and transform the code above into this:
```{.cpp}
int main()
{
	//Create robot and start control loop
	franka_timeout_handler::Robot robot("192.168.0.1");
	robot.start(franka_timeout_handler::ControllerType::cartesian);
	//Wait for call
	robot.receive();
	//The call occured, now `main()` has ~300 microseconds to analyze and change `robot` object
	// ...modifiying `robot`...
	//Send values back
	robot.send();
	//It might have taken longer then ~300 microseconds between receive() and send()
	//But even in this case nothing critical happens, the robot simply applies previous parameter values in background
	printf(robot.get_late() ? "Late" : "Not late");
}
```

Fields of `Robot` class can be divided in three groups:

 - Input: fields are refreshed with `receive()` (positions, velocities, etc.)

 - Output: fields are applied with `send()` (targets, parameters, etc.)

 - Result: fileds are refreshed with `send()` (torques and lateness indicator)

Note that setters **do not** actually set values the controller uses. `send()` or its variations must be called for that.

The simplest `CmakeLists.txt` that uses `franka_timeout_handler` reads:
```
project(example)
cmake_minimum_required(VERSION 3.14.0)
find_package(franka_timeout_handler 1.0.0 REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE franka_timeout_handler)
```

@section Dependencies
The library depends on:
 - [libfranka](https://github.com/frankaemika/libfranka) (set with `-Dfranka_DIR=/absolute_path_to_libfranka/build` or as part of `ROS`)
 - [Eigen](https://eigen.tuxfamily.org)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [CMake](https://cmake.org) >= `3.14.0`
 - Fully preemptable Linux kernel
 - C++11 compatible compiler

@section Building
`franka_timeout_handler` can be built with [CMake](https://cmake.org) using following commands:
```
mkdir build
cd build
cmake ..
cmake --build .
```

@section Installation
```
mkdir build
cd build
cmake ..
cmake --build .
sudo cmake --install .
sudo ldconfig
#Further steps are required only if you plan to use pybind'ded classes from franka_timeout_handler in your pybind'ded library
cmake .. -Dfranka_timeout_handler_omit_include_directories=yes
sudo cmake --build .
cmake --install .
```

@section Documentation
`Python` docstrings are provided. `C++` code is documented with comments. [Doxygen](https://www.doxygen.nl) documentation may be generated with `doxygen` command.

@section Contributors
 - Kyrylo Sovailo
 - Sukhija Bhavi
*/
