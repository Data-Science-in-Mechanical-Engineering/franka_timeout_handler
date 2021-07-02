# Welcome to Franka real-time
Here you will find a small and simple library that allows non-real-time programs to operate with Franka Panda robot at hight frequency.

### Contents
1. [Welcome to Franka real-time](#welcome-to-franka-real-time)
2. [Contents](#contents)
4. [Usage](#usage)
4. [Dependencies](#dependencies)
5. [Installation](#installation)
6. [Documentation](#documentation)

### Usage
In order to understand what this library does, it may be helpful to take a look at an example of robot control with [libfranka](https://github.com/frankaemika/libfranka):
```c++
franka::Robot robot("192.168.0.1");
robot.control([](const franka::RobotState &robot_state, franka::Duration time) -> franka::Torques
{
	//Robot called our function, we have have ~300 microseconds to analyze `robot_state` and return torques
	return calculate_torques_from_robot_state(robot_state);
});
```
*for `Python` programmers: here a multi-line lambda function that accepts state and returns torques, is passed `robot.control` function*

Direct `Python` wrapper of this code is possible, but if `Python` freezes (for example because of garbage collector), an error would occur. The library's goal is to avoid this situation and transform the code above into this:
```c++
franka_real_time::Robot robot("192.168.0.1");
robot.start_cartesian();
//We started control loop, now we need to wait
robot.receive();
//The robot called internal function, we have ~300 microseconds to analyze and change `robot`
robot.send();
//We might have exceeded the timeout and our changes might be impossible to apply, but nothing critical happened to the robot
printf(robot.late ? "Late" : "Not late");
```

Fields of `Robot` class can be divided in three groups:

 - Input: fields are refreshed with `receive()` (positions, velocities, etc.)

 - Output: fields are applied with `send()` (stiffness, damping, etc.)

 - Result: fileds are refreshed with `send()` (torques and lateness indicator)

### Dependencies
The library depends on:
 - [libfranka](https://github.com/frankaemika/libfranka) (as part of ROS)
 - [Eigen](https://eigen.tuxfamily.org)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [CMake](https://cmake.org) >= `3.10.2`
 - Fully preemptable Linux kernel
 - C++11 compatible compiler

### Installation
```
mkdir build
cd build
cmake ..
cmake --build .
```

### Documentation
[Doxygen](https://www.doxygen.nl) documentation is provided.