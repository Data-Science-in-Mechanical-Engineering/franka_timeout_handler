#pragma once

#include <franka/gripper.h>
#include <thread>
#include <string>

namespace franka_timeout_handler
{
    ///Simple class for robot's gripper. It is made for reasons of completenes and autonomy, and is not capable of any timeout handling.
	class Gripper
	{
    private:
        franka::Gripper *_gripper;
        std::thread _thread;
        bool _started = false;

    public:
        ///Creates gripper
        Gripper(std::string ip);
        ///Calibrates gripper's fingers
        bool homing();
        ///Returns fingers' width
        double get_width();
        ///Returns if fingers are grasped, i.e. (width - epsillon_inner) < real_width < (width + epsillon_outer)
        bool get_grasped();
        ///Returns fingers' temperature
        double get_temperature();
        ///Moves the fingers to a specified width with specified speed
        bool move(double width, double speed);
        ///Opens or closes fingers with specified force and speed
        bool grasp(double width, double speed, double force, double epsilon_inner = 0.005, double epsilon_outer = 0.005);
        ///Returns if asyncronous comand is currently running
        bool async_started();
        ///Waits completiting of asyncronous command
        void async_wait();
        ///Moves the fingers to a specified width with specified speed in asyncronous mode
        void async_move(double width, double speed);
        ///Opens or closes fingers with specified force and speed in asyncronous mode
        void async_grasp(double width, double speed, double force, double epsilon_inner = 0.005, double epsilon_outer = 0.005);
        ///Destroys gripper
        ~Gripper();
	};
}