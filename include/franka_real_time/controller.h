#pragma once

namespace franka_real_time
{
    ///Abstract controller
    class Controller
    {
    public:
        virtual void receive()          = 0;
		virtual void send()             = 0;
		virtual void receive_and_send() = 0;
		virtual void send_and_receive() = 0;
    };
}