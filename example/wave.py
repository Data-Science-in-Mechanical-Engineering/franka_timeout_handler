#!/bin/python3

import sys
import os
import math
import franka_timeout_handler
import numpy as np

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./hold.py IP\n")
    else:
        try:
            robot = franka_timeout_handler.Robot(sys.argv[1])
            robot.start(franka_timeout_handler.ControllerType.cartesian)
            target_position = franka_timeout_handler.default_target_position()
            target_position[2] = target_position[2] + 0.1
            robot.move_target_position_wxyz(target_position, franka_timeout_handler.default_target_orientation_wxyz(), 3000)
            robot.receive()
            call0 = robot.get_call()
            time = 10000
            period = 1000
            late = 0
            for i in range(time):
                robot.receive()
                call = robot.get_call() - call0
                target_position = franka_timeout_handler.default_target_position()
                target_position[1] = target_position[1] + 0.1 * math.sin(2 * math.pi * call / period)
                target_position[2] = target_position[2] + 0.1 * math.cos(2 * math.pi * call / period)
                robot.set_target_position(target_position)
                robot.send()
                print("Late" if robot.get_late() else "Not late")
                if robot.get_late(): late = late + 1
            for i in range(1000): robot.receive_and_send()
            print("Total: late ", late, " times of ", time, " (", 100.0 * late / time, "%)")
        except:
            print(sys.exc_info()[0])