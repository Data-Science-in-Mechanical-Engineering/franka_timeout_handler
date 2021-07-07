import sys
import os
import franka_real_time
import numpy as np
state=[]
def read_state(robot,k):
	call=k
	if call%5==0:
		pos=robot.get_position()
		vel=robot.get_velocity()
		state.append(np.vstack((pos.reshape(-1,1),vel.reshape(-1,1))))


if __name__ == "__main__":
	if len(sys.argv) != 2:
		print("Usage: ./hold IP\n")
		exit()

	robot = franka_real_time.Robot(sys.argv[1])
	robot.start()
	robot.move_to_reference()
	r=0.125
	ellipse_factor=1.25
	robot.receive()
	center=robot.get_position()
	robot.send()
	Next_target=center.copy()
	reference=center.copy()
	reference[1]=ellipse_factor*r+center[1]
	traj_time=2000
	i=0
	while(i<traj_time):
		Next_target=center+i*1/traj_time*(reference-center)
		robot.set_target_position(Next_target)
		robot.receive_and_send()
		if not robot.get_late():
			i+=1
	
	for i in range(500):
		robot.receive_and_send()	
	#for i in range(traj_time):
	#	Next_target=center+i*1/traj_time*(reference-center)
	#	robot.set_target_position(Next_target)
	#	robot.receive_and_send()
	w=np.pi/2000
	k=0
	default_stiffness=robot.get_translation_stiffness()
	default_damping=robot.get_translation_damping()
	stiffness=0.1*default_stiffness.copy()
	damping=0.32*default_damping.copy()
	robot.set_translation_stiffness(stiffness)
	robot.set_translation_damping(damping)
	while(k<20000):
		if k==10000:
			read_state(robot,k)
			robot.set_translation_stiffness(default_stiffness)
			robot.set_translation_damping(default_damping)
			Next_target[0]=center[0]+r*np.sin(k*w+np.pi)
			Next_target[1]=center[1]+ellipse_factor*r*np.cos(k*w)
			#print(Next_target)
			robot.set_target_position(Next_target)
			
			robot.receive_and_send()
			if not robot.get_late():
				#print("Updated_k")
				k+=1
			continue
		
		read_state(robot,k)
		Next_target[0]=center[0]+r*np.sin(k*w+np.pi)
		Next_target[1]=center[1]+ellipse_factor*r*np.cos(k*w)
		#print(Next_target)
		robot.set_target_position(Next_target)
		robot.receive_and_send()
		if not robot.get_late():
			#print("Updated_k")
			k+=1
	print(np.asarray(state).shape)
	#for k in range(traj_time):
	#	Next_target[0]=center[0]+r*np.sin(k*w+np.pi)
	#	Next_target[1]=center[1]+r*np.cos(k*w)
		#print(Next_target)
	#	robot.set_target_position(Next_target)
	#	robot.receive_and_send()
	#while True:
	#	traj[0]=center[0]+r*np.sin(k*w+np.pi)
	#	traj[1]=center[1]+r*np.cos(k*w)
	#	k+=1
	#	robot.receive()
	#	robot.set_target_position(traj)
	#	robot.send()
	#	#robot.receive_and_send()
	#	print("Late\n" if robot.get_late() else "Not late\n")
