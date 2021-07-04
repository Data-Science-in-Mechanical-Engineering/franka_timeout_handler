import sys
import os
import franka_real_time
import numpy

if __name__ == "__main__":
	if len(sys.argv) != 2:
		print("Usage: ./hold IP\n")
		exit()

	robot = franka_real_time.Robot(sys.argv[1])
	robot.start()
	while True:
		robot.receive()
		robot.send()
		print("Late\n" if robot.get_late() else "Not late\n")