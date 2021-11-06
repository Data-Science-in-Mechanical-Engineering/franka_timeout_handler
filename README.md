# Welcome to `Franka_timeout_handler 1.2.0`!
Here you will find a library for Franka Emika Panda robot, which allows you to send messages to real-time controller from non-real-time application without breaking robot's real-time requirements.

### Contents
1. [Welcome to Franka_timeout_handler](#welcome-to-Franka_timeout_handler)
2. [Contents](#contents)
4. [Usage](#usage)
4. [Dependencies](#dependencies)
5. [Building](#building)
6. [Installation](#installation)
7. [Documentation](#documentation)
8. [Contributors](#contributors)

### Usage
In order to understand what this library does, it may be helpful to take a look at an example of robot control with [libfranka](https://github.com/frankaemika/libfranka):
```c++
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
```c++
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

The simplest `CmakeLists.txt` that uses `Franka_timeout_handler` reads:
```
project(example)
cmake_minimum_required(VERSION 3.14.0)
find_package(Franka_timeout_handler 1.0.0 REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE Franka_timeout_handler)
```

### Dependencies
The library depends on:
 - [libfranka](https://github.com/frankaemika/libfranka) (set with `-DFranka_DIR=/absolute_path_to_libfranka/build` or as part of `ROS`)
 - [Eigen](https://eigen.tuxfamily.org)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [CMake](https://cmake.org) >= `3.14.0`
 - Fully preemptable Linux kernel
 - C++11 compatible compiler

### Building
`Franka_timeout_handler` can be built with [CMake](https://cmake.org) using following commands:
```
mkdir build
cd build
cmake ..
cmake --build .
```

### Installation
```
mkdir build
cd build
cmake ..
cmake --build .
sudo cmake --install .
sudo ldconfig
#Further steps are required only if you plan to use pybind'ded classes from Franka_timeout_handler in your pybind'ded library
cmake .. -DFranka_timeout_handler_omit_include_directories=yes
sudo cmake --build .
cmake --install .
```

### Documentation
`Python` docstrings are provided. `C++` code is documented with comments. [Doxygen](https://www.doxygen.nl) documentation may be generated with `doxygen` command.

### Contributors
 - Kyrylo Sovailo
 - Sukhija Bhavi
