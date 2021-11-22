#!/usr/bin/env python3
import rospy
import time
import numpy as np
from geometry_msgs.msg import *
from std_msgs.msg import Float32, Float32MultiArray, String, Bool
#from termcolor import colored

#Drone Status Node
#Proyecto Especial

pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, pathTime, endPos_x, endPos_y, endPos_z = 0,0,0,0,0,0,0,0,0,0,0

simTime, realTime = 0, 0

force, torque, tankMass, tankVolume, velocity = [],0,0,' ',0

restartTank = False

#Obtenemos la posicion del dron
def dronePose_callback(msg):
    global pos_x, pos_y, pos_z
    #_,pos=sim.simxGetObjectPosition(clientID, Quadricopter_base, -1, sim.simx_opmode_blocking)
    pos_x = msg.data[0]
    pos_y = msg.data[1]
    pos_z = msg.data[2]
    return pos_x, pos_y, pos_z

#Obtenemos la orientacion del dron
def droneOrientation_callback(msg):
    global ang_x, ang_y, ang_z
    #_, orientation = sim.simxGetObjectOrientation(clientID,Quadricopter_base,-1,sim.simx_opmode_blocking)
    ang_x = msg.data[0]
    ang_y = msg.data[1]
    ang_z = msg.data[2]
    return ang_x, ang_y, ang_z

#Estructura : [rho, pathTime, EndPos]
def status_callback(msg):
	global rho, pathTime, endPos_x, endPos_y, endPos_z
	
	rho =  msg.data[0]
	pathTime = msg.data[1]
	endPos_x, endPos_y, endPos_z = msg.data[2], msg.data[3], msg.data[4]

#Estructura : [delta_realTime, delta_simTime]
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

def info_status():
	global pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, pathTime,  endPos_x, endPos_y, endPos_z
	global simTime, realTime, force, torque, tankMass, tankVolume, velocity, restartTank

	rospy.init_node('Status_Node', anonymous=True)  # Inicia el nodo status

	#Subscripcion a topicos
	rospy.Subscriber("/force", Float32MultiArray, force_callback, tcp_nodelay=True)
	rospy.Subscriber("/torque", Float32MultiArray, torque_callback, tcp_nodelay=True)
	rospy.Subscriber('/velocity', Float32MultiArray, velocity_callback, tcp_nodelay=True)
	#rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
	#rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
	rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
	rospy.Subscriber('/PE/Drone/tank_volume', String, tankVolume_callback)
	rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
	rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)

	#Estructura : [rho, pathTime, Endpos]
	rospy.Subscriber("/PE/Drone/drone_status", Float32MultiArray, status_callback, tcp_nodelay=True)
	#Estructura : [delta_realTime, delta_simTime]
	rospy.Subscriber("PE/Drone/controller_time", Float32MultiArray, times_callback, tcp_nodelay=True)
	rospy.Subscriber('/PE/Drone/restart', Bool, callback_restart)
	rate = rospy.Rate(10)
	rpm = 0
	rpm_list = [0,0,0,0]

	f = open("flight_data.txt", "w")

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

		for motor in range(len(force)):
			#print('motor #',motor)
			#print('force #',force[motor])
			rpm = np.sqrt(abs(force[motor])/((1/16)*p*np.pi*(R**4)*(e_d**4)*C_T))
			rpm_list[motor] = rpm

		mensaje = '_________________________________________________________ \n \n'
		mensaje = mensaje + 'Pose Actual: ' + str(round(pos_x,4)) + ', ' + str(round(pos_y,4)) + ', ' + str(round(pos_z,4)) + ' Angulo Actual: ' + str(round(ang_x,3)) + ', ' + str(round(ang_y,3)) + ', ' + str(round(ang_z,3)) + '\n'
		mensaje = mensaje + 'Pose Final: ' + str(round(endPos_x,3)) + ', ' + str(round(endPos_y,3)) + ', ' + str(round(endPos_z,3)) + '\n'
		mensaje = mensaje + '\n'
		mensaje = mensaje + 'Rho: ' + str(round(rho,3)) + '\n'
		mensaje = mensaje + '\n'
		mensaje = mensaje + 'Motor Forces: ' + str(force) + '\n \n'#+ str(round(force[0],3)) + ', '+ str(round(force[1],3))+ ', '+ str(round(force[2],3))+ ', '+ str(round(force[2],3))+ '\n'
		mensaje = mensaje + 'Motor Torque: ' + str(torque) + '\n \n'
		mensaje = mensaje + 'Motor RPM: ' + str(rpm_list) + '\n'
		mensaje = mensaje + 'Tank Mass: ' + str(round(tankMass,3)) + ' Kg, Tank Volume: ' + tankVolume + ' Tank Restart: ' + str(restartTank) +'\n'
		mensaje = mensaje + '\n'
		mensaje = mensaje + 'RealTime: ' + str(round(realTime,4)) +' simTime: ' + str(round(simTime,4)) + ' PathTime: ' + str(round(pathTime,3))
		mensaje = mensaje + '\n \n'

		if tankMass == 0:
			print('tankMass == 0, program wont start')
		else:
			print(mensaje)
			f.write(str(force) + str(torque) + str(rpm_list))
			
	
		time.sleep(1)

		#rate.sleep()
	f.close()

if __name__== '__main__':
	try:
		info_status()
	except rospy.ROSInterruptException:
		print('Nodo detenido')