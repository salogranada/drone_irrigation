#!/usr/bin/env python3
from sys import path_hooks
import rospy
import time
import numpy as np
from geometry_msgs.msg import *
from std_msgs.msg import Float32, Float32MultiArray, String, Bool
import os
#from termcolor import colored

#Drone Status Node
#Prints in terminal everything to monitor the drone movement
#Calculates each motors RPM for ML trainning.
#Saves data to file.

#****************************************
#FOR ANY NEW PATH SIMULATION YOU HAVE TO SPECIFY SAVING 2 FILES! flight data and error report.
#****************************************


#Author: Salomón Granada Ulloque
#Email: s.granada@uniandes.edu.co

pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, pathTime, endPos_x, endPos_y, endPos_z = 0,0,0,0,0,0,0,0,0,0,0

simTime, realTime = 0, 0

force, torque, tankMass, tankVolume, velocity = [],0,0,' ',0

restartTank = False
path_vel, route_num, missing_points = 0,0,0
droneVel, target_rho = 0, 0

#Real-time drone position callback
def dronePose_callback(msg):
    global pos_x, pos_y, pos_z
    #_,pos=sim.simxGetObjectPosition(clientID, Quadricopter_base, -1, sim.simx_opmode_blocking)
    pos_x = msg.data[0]
    pos_y = msg.data[1]
    pos_z = msg.data[2]
    return pos_x, pos_y, pos_z

#Real-time drone orientation callback
def droneOrientation_callback(msg):
    global ang_x, ang_y, ang_z
    #_, orientation = sim.simxGetObjectOrientation(clientID,Quadricopter_base,-1,sim.simx_opmode_blocking)
    ang_x = msg.data[0]
    ang_y = msg.data[1]
    ang_z = msg.data[2]
    return ang_x, ang_y, ang_z

#Estructure : [rho, pathTime, EndPos, path_vel, route No., missing_points, dronevel]
def status_callback(msg):
	global rho, pathTime, endPos_x, endPos_y, endPos_z, path_vel, route_num, missing_points, droneVel, target_rho
	
	rho =  msg.data[0]
	pathTime = msg.data[1]
	endPos_x, endPos_y, endPos_z = msg.data[2], msg.data[3], msg.data[4]
	path_vel = msg.data[5]
	route_num = msg.data[6]
	missing_points = msg.data[7]
	droneVel = msg.data[8]
	target_rho = msg.data[9]

#Estructure : [delta_realTime, delta_simTime]
def times_callback(msg):
	global simTime, realTime
	realTime = msg.data[0]
	simTime = msg.data[1]

def force_callback(msg):
    global force
    force = [msg.data[0],msg.data[1],msg.data[2],msg.data[3]]

def torque_callback(msg):
    global torque
    torque = [msg.data[0],msg.data[1],msg.data[2],msg.data[3]]

def tankMass_callback(msg):
    global tankMass
    tankMass = msg.data/1000

def velocity_callback(msg):
	global velocity
	feedback_vel_x = msg.data[0]
	feedback_vel_y = msg.data[1]
	
	velocity = np.sqrt(feedback_vel_x**2 + feedback_vel_y**2)

def tankVolume_callback(msg):
	global tankVolume
	tankVolume = msg.data

def callback_restart(msg):
	global restartTank
	restartTank = msg.data

#Prints into terminal all information requiered for monitoring the drone.
#Also writes into file forces, torques and RPMs for ML model further trainning.
def info_status():
	global pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, pathTime,  endPos_x, endPos_y, endPos_z, droneVel, target_rho
	global simTime, realTime, force, torque, tankMass, tankVolume, velocity, restartTank, path_vel, route_num, missing_points

	rospy.init_node('Status_Node', anonymous=True)  # Inicia el nodo status

	#Subscripcion a topicos
	rospy.Subscriber("/force", Float32MultiArray, force_callback, tcp_nodelay=True)
	rospy.Subscriber("/torque", Float32MultiArray, torque_callback, tcp_nodelay=True)
	rospy.Subscriber('/velocity', Float32MultiArray, velocity_callback, tcp_nodelay=True)
	rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
	rospy.Subscriber('/PE/Drone/tank_volume', String, tankVolume_callback)
	rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
	rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)

	#Estructure : [rho, pathTime, Endpos, path_vel, route No., missing_points, dronevel]
	rospy.Subscriber("/PE/Drone/drone_status", Float32MultiArray, status_callback, tcp_nodelay=True)
	#Estructure : [delta_realTime, delta_simTime]
	rospy.Subscriber("PE/Drone/controller_time", Float32MultiArray, times_callback, tcp_nodelay=True)
	rospy.Subscriber('/PE/Drone/restart', Bool, callback_restart)
	rate = rospy.Rate(10)

	terminal_msg = ''
	rpm = 0
	rpm_list = [0,0,0,0]
	cum_posx = 0
	cum_posy = 0
	prevX, xVel = 0,0
	prevY, yVel = 0,0
	saveDistX = 0
	saveDistY = 0

	#Opens file for savig data from flight
	scriptDir = os.path.dirname(__file__)
	flight_data = scriptDir +'/../data_base/flight_data/prueba_paths.txt'
	f = open(flight_data, "w")
	f.write('route_num|missing_points|simTime|tankMass|force_array|torque_array|rpm_list|droneVel|error_target_vel|error_target_dist|xDisplacement|yDisplacement|x_avg_Vel|y_avg_Vel \n')
	
	#opens file for error log.
	error_report = scriptDir +'/../data_base/reports/prueba_paths.txt'
	error_file = open(error_report, "w")
	error_file.write('route_num|point|simTime|tankMass|RHO|target_rho|log \n')

	initial_points = missing_points

	while not rospy.is_shutdown():

		#RPM for each motor calculation. 
		c = 0.6
		e_d = 0.88 #diameter effectiveness
		theta = 25 #blade twist angle (deg)
		p = 1.225 #air density kg/m^3
		R = (1.4974e-01)/2 #blade radio
		k = 2*c/4*R #Motor-propeller Force Constant
		C_T = (4/3)*k*theta*(1-(1-e_d)**3) - k*(np.sqrt(k*(k+1))- np.sqrt(k))*(1-(1-e_d)**2)

		#T = (1/16)*p*np.pi*(R**4)*(e_d**4)*C_T*(rpm**2)

		#For each of the motors calculates its RPM depending on the exerted force.
		for motor in range(len(force)):
			rpm = np.sqrt(abs(force[motor])/((1/16)*p*np.pi*(R**4)*(e_d**4)*C_T))
			rpm_list[motor] = rpm

		#Total axis movement calculation. How much does the drone moves in each axis.
		if initial_points == missing_points:
			#terminal_msg = terminal_msg + '*********************adding positions*********************'

			dist_x = abs(prevX-pos_x)
			dist_y = abs(prevY-pos_y)

			cum_posx = cum_posx + dist_x
			cum_posy = cum_posy + dist_y

			prevX = pos_x
			prevY = pos_y
			saveTime = simTime

		else:
			saveDistX = cum_posx
			saveDistY = cum_posy
			if saveTime!=0:
				xVel = saveDistX/saveTime
				yVel = saveDistY/saveTime

			cum_posx, cum_posy = 0,0
			initial_points = missing_points
			
		#Builds printed message in terminal
		terminal_msg = terminal_msg + '__________________________________________________________________ \n \n'
		terminal_msg = terminal_msg + 'Pose Actual: ' + str(round(pos_x,4)) + ', ' + str(round(pos_y,4)) + ', ' + str(round(pos_z,4)) + ' Angulo Actual: ' + str(round(ang_x,3)) + ', ' + str(round(ang_y,3)) + ', ' + str(round(ang_z,3)) + '\n'
		terminal_msg = terminal_msg + 'Pose Final: ' + str(round(endPos_x,3)) + ', ' + str(round(endPos_y,3)) + ', ' + str(round(endPos_z,3)) + '   Path Vel: '+str(round(path_vel,3)) + '\n \n'
		terminal_msg = terminal_msg + 'Rho: ' + str(round(rho,3)) + '   target_rho: ' + str(round(target_rho,3)) +'  Route num: '+ str(route_num) + '  Missing_points: '+ str(missing_points)  +'\n \n'
		terminal_msg = terminal_msg + 'Drone_Vel: ' + str(round(droneVel,3)) + '   Axis Vel: '+ str(round(xVel,3)) +' - ' + str(round(yVel,3))+ '   Displacement: '+ str(round(saveDistX,3)) +' - ' + str(round(saveDistY,3)) + '\n \n'

		terminal_msg = terminal_msg + 'Motor Forces: ' + str(force) + '\n \n'#+ str(round(force[0],3)) + ', '+ str(round(force[1],3))+ ', '+ str(round(force[2],3))+ ', '+ str(round(force[2],3))+ '\n'
		terminal_msg = terminal_msg + 'Motor Torque: ' + str(torque) + '\n \n'
		terminal_msg = terminal_msg + 'Motor RPM: ' + str(rpm_list) +'\n \n'
		terminal_msg = terminal_msg + 'Tank Mass: ' + str(round(tankMass,3)) + ' Kg, Tank Volume: ' + tankVolume + ' Tank Restart: ' + str(restartTank) +'\n \n'

		terminal_msg = terminal_msg + 'RealTime: ' + str(round(realTime,4)) +' simTime: ' + str(round(simTime,4)) + ' PathTime: ' + str(round(pathTime,3))
		terminal_msg = terminal_msg + '\n \n'

		#Only print if the tank mass calculation is actually working.
		if tankMass == 0:
			print('tankMass == 0, program wont start')
		else:
			print(terminal_msg)
			error_target_vel = path_vel - droneVel
			error_target_dist = target_rho

			#Write in flight_data file.
			f.write(str(route_num) + '|' + str(missing_points) + '|' + str(simTime) + '|' + str(tankMass) + '|' + str(force) + '|'+ str(torque)  +'|'+ str(rpm_list)+ '|' + str(droneVel) + '|' + str(error_target_vel) + '|' + str(error_target_dist)+ '|' + str(saveDistX)+ '|' + str(saveDistY)+ '|' + str(xVel)+ '|' + str(yVel) +'\n')
			
			#Write in error reports file.
			if simTime < 0:
				#error_file.write('In Route: ' + str(route_num)+ ' In point: ' + str(missing_points) + ' SimTime: ' + str(round(simTime,4)) + ' TankMass: ' + str(round(tankMass,4))+  ' |Negative Simtime \n')
				error_file.write(str(route_num) + '|' + str(missing_points) + '|' + str(simTime) + '|' + str(tankMass) + '|' + str(rho) + '|'+ str(target_rho) + '|Negative Simtime \n')

			if rho > 15:
				#error_file.write('In Route: ' + str(route_num)+ ' In point: ' + str(missing_points) + ' SimTime: ' + str(round(simTime,4)) + ' RHO: ' + str(round(rho,4)) + ' |TARGET out of control and lost the WAYPOINT \n')
				error_file.write(str(route_num) + '|' + str(missing_points) + '|' + str(simTime) + '|' + str(tankMass) + '|' + str(rho) + '|'+ str(target_rho) + '|TARGET out of control and lost the WAYPOINT \n')

			elif target_rho > 2:
				error_file.write(str(route_num) + '|' + str(missing_points) + '|' + str(simTime) + '|' + str(tankMass) + '|' + str(rho) + '|'+ str(target_rho) + '|DRONE out of control and lost the TARGET \n')

		time.sleep(1)
	f.close()
	

if __name__== '__main__':
	try:
		info_status()
	except rospy.ROSInterruptException:
		print('Nodo detenido')