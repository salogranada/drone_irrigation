#! /usr/bin/env python3
import rospy
import sys
import os
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, String, Bool
import time

#Drone control guided by exerted force in previous try.
#Reads paths from flight data file.
#Coppeliasim Scene: tracking_control.ttt

#****************************************
#FOR ANY SIMULATION YOU HAVE TO SPECIFY THE PATH FILE!
#****************************************
paths_file = '/../data_base/flight_data/fd_pathprueba_1' #Input FILE NEEDS TO HAVE HEADER

#Author: Salomón Granada Ulloque
#Email: s.granada@uniandes.edu.co

pos_x, pos_y, pos_z, theta, deltaX, deltaY, posTarget_x,posTarget_y,posTarget_z = 0, 0, 0, 0, 0, 0, 0, 0, 0
ang_x, ang_y, ang_z = 0,0,0

rho = 0

#Time for each of the paths in the trayectory [s] TEST
tiempo = [80, 40, 30, 20]

#Path points inside of the trayectory [m] TEST
ruta = [[-4,0,1], [-3,0,1], [0,0,1], [2,0,1], [0,4,1]]

simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0
tankMass = 0
init = False

#Particle (target) position
def targetPose_callback(msg):
    global posTarget_x, posTarget_y, posTarget_z
    posTarget_x = msg.data[0]
    posTarget_y = msg.data[1]
    posTarget_z = msg.data[2]

#Drone position
def dronePose_callback(msg):
    global pos_x, pos_y, pos_z
    pos_x = msg.data[0]
    pos_y = msg.data[1]
    pos_z = msg.data[2]
    return pos_x, pos_y, pos_z

#Drone orientation
def droneOrientation_callback(msg):
    global ang_x, ang_y, ang_z
    ang_x = msg.data[0]
    ang_y = msg.data[1]
    ang_z = msg.data[2]
    return ang_x, ang_y, ang_z

#OSimulation time:
def simTime_callback(msg):
    global simTime
    simTime = msg.data

#Obtenemos tiempo real
def realTime_callback(msg):
    global realTime
    realTime = msg.data

#Tank mass weight
def tankMass_callback(msg):
    global tankMass
    tankMass = msg.data


#Main function for drone movement.
def main_control():
    global pos_x, pos_y, pos_z, theta, deltaX, deltaY, tiempo, ruta, ang_x, ang_y, ang_z, posTarget_x,posTarget_y,posTarget_z
    global simTime_actual, realTime_actual, simTime_anterior, realTime_anterior, realTime, simTime, tankMass, init, paths_file

    print('Starting control node...')
    print(' ')
    rospy.init_node('Control_Node', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    #Topic publishing
    pub_redoforces = rospy.Publisher('/PE/Drone/redo_forces', Float32MultiArray, queue_size=10)

    #Topic subscribers:
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
    rospy.Subscriber("/target_pose", Float32MultiArray, targetPose_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)

    #paths_file = input('Input paths file (no extention) >')
    scriptDir = os.path.dirname(__file__)
    paths_file = scriptDir + paths_file + '.txt'
    file = open(paths_file)
    file.readline() #Skip header

    redo_forces = Float32MultiArray()
    
    while not rospy.is_shutdown():

        line = file.readline() #Reads first path

        #Go over each line of the paths file
        while line != '':
            linea = line.split('|')

            #Get each of the points in the path file
            ruta_list = linea[4].split(',')
            intruta_list = []

            for i in ruta_list:
                intPoints = float(i) #convert time strings to int
                intruta_list.append(intPoints)

            #Update simulation and real time.
            simTime_actual = simTime
            realTime_actual = realTime

            delta_simTime = simTime_actual - simTime_anterior
            delta_realTime = realTime_actual - realTime_anterior

            simTime_anterior = simTime_actual
            sim_anterior2 = simTime_actual #helps checking if 1 second has passed in simulation.
            realTime_anterior = realTime_actual

            simTime_actual = simTime
            realTime_actual = realTime

            delta_simTime = simTime_actual - simTime_anterior

            time.sleep(1)

            if simTime_actual - sim_anterior2 < 0:
                sim_anterior2 = 0
            if delta_simTime < 0 :
                simTime_anterior = 0
            
            print(simTime_actual, '  anterior: ',sim_anterior2 )
            print('forces: ', intruta_list)
            redo_forces.data = intruta_list
            pub_redoforces.publish(redo_forces)
            
            #Only when 1 second has passed in simulation, publish the new position.
            #Simulation can go faster than real time and still behave as supposed.
            if simTime_actual - sim_anterior2 >= 1:

                print('----------FORCES: ', ruta_list)
                sys.stdout.write("\033[K") # Clear to the end of line
                sys.stdout.write("\033[F") # Cursor up one line

                pub_redoforces.publish(redo_forces)

                sim_anterior2 = simTime_actual

            delta_simTime = simTime_actual - simTime_anterior
            delta_realTime = realTime_actual - realTime_anterior
            

            simTime_anterior = 0
            delta_realTime, delta_simTime = 0,0
            line = file.readline() #Read next path
            print('**************************Read next line*********************')
            
        print('----------------------------Finished Trayectory--------------------------')
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line
        rate.sleep()

if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Stoped node.')