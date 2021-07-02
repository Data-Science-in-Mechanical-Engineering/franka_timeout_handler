#pragma once

namespace franka_real_time
{
    class Robot;

    ///Abstract controller
    class Controller
    {
    friend Robot;
    protected:
        enum class Type
        {
            cartesian
        };

        virtual Type typ() const        = 0;
        virtual void receive()          = 0;
		virtual void send()             = 0;
		virtual void receive_and_send() = 0;
		virtual void send_and_receive() = 0;
    };
}