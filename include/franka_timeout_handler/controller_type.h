#pragma once

namespace franka_timeout_handler
{
    ///Type of the controller
    enum class ControllerType
    {
        cartesian,  ///< Drives robot's hand to cartesian target, ignores joint target
        joint       ///< Drives robot's joints to joint target, ignores cartesian target
    };
}
