#pragma once

namespace franka_timeout_handler
{
    ///Update policy of controller parameters
    enum class Update
    {
        never,      ///< Parameter is never updated
        always,     ///< If `send()` is not late, parameter is updated immediately. Otherwise, it is updated in next cycle
        if_not_late ///< Parameter is updated only if  `send()` is not late
    };
}