#pragma once

#include "controller.h"
#include <franka/robot.h>
#include <franka/model.h>

namespace franka_real_time
{
    class Controller;

	///Franka Panda robot
	class Robot
	{
    friend Controller;
    private:
        franka::Robot _robot;
        franka::Model *_model = nullptr;
        Controller *_controller;

	public:
        ///Creates robot
		Robot(std::string ip);
		///Destroys robot
		~Robot();
	};
}