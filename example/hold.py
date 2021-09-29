#!/bin/python3

import sys
import os
import franka_timeout_handler
import numpy

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./hold.py IP")
    else:
        try:
            robot = franka_timeout_handler.Robot(sys.argv[1])
            robot.start(franka_timeout_handler.ControllerType.cartesian)
            time = 10000
            late = 0
            for i in range(time):
                robot.receive()
                robot.send()
                print("Late" if robot.get_late() else "Not late")
                if robot.get_late(): late = late + 1
            print("Total: late ", late, " times of ", time, " (", 100.0 * late /  time, "%)")
        except:
            print(sys.exc_info()[0])