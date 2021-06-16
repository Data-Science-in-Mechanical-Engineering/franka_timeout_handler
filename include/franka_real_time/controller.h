#pragma once

#include "robot.h"
#include <franka/robot.h>
#include <franka/model.h>

namespace franka_real_time
{
    class Robot;

    ///Abstract controller
    class Controller
    {
    protected:
        Robot *_robot = nullptr;
        static void _set_controller(Robot *robot, Controller *controller);
        franka::Robot *_get_robot();
        franka::Model *_get_model();
    };
}