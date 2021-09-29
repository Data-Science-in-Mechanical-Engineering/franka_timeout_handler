#pragma once

namespace franka_timeout_handler
{
    ///Type of the controller
    enum class ControllerType
    {
        cartesian,  ///< Makes robot try to approach given cartesian position
        joint       ///< Makes robot try to approach given joint positions
    };
}
