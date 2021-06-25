import sys
import os
import franka_real_time


if __name__ == "__main__":
	if len(sys.argv) != 2:
		print("Usage: ./hold IP\n")
		exit()

	franka_real_time.Robot robot(sys.argv[1])
	robot.control_cartesian()
    
	robot.receive()
	robot.target_position = robot.position
	robot.target_orientation = robot.orientation
    
	while True:
		robot.receive()
		robot.send()
		print("Late\n" if robot.late else "Not late\n")