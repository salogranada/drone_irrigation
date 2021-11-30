#! /usr/bin/env python3
import rospy
import sys
import os
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, String, Bool
import sim
import time

#Drone control guided by particle in simulation.
#Set the list of points in the trajectory and time the drone must reach each point.
#Reads paths from file.
#Coppeliasim Scene: tracking_control.ttt

#Author: Salomón Granada Ulloque
#Email: s.granada@uniandes.edu.co

def connect(port):
    # Establece la conexion a VREP
    # port debe coincidir con el puerto de conexion en VREP
    # retorna el numero de cliente o -1 si no puede establecer conexion
    sim.simxFinish(-1) # just in case, close all opened connections\n",
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado a", port)
    else: print("No se pudo conectar")
    return clientID

#Conectarse al servidor de VREP
clientID = connect(19998)


pos_x, pos_y, pos_z, theta, deltaX, deltaY, posTarget_x,posTarget_y,posTarget_z = 0, 0, 0, 0, 0, 0, 0, 0, 0
ang_x, ang_y, ang_z = 0,0,0

rho = 0

#Time for each of the paths in the trayectory [s]
tiempo = [80, 40, 30, 20]

#Path points inside of the trayectory [m]
ruta = [[-4,0,1], [-3,0,1], [0,0,1], [2,0,1], [0,4,1]]

simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0
tankMass = 0
init = False

print('Program started')

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
    global simTime_actual, realTime_actual, simTime_anterior, realTime_anterior, realTime, simTime, tankMass, init

    print('Starting control node...')
    print(' ')
    rospy.init_node('Control_Node', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    #Topic publishing
    pub_pose = rospy.Publisher('/drone_nextPose', Float32MultiArray, queue_size=10)
    pub_euler = rospy.Publisher('/drone_nextEuler', Float32MultiArray, queue_size=10)
    pub_tank_volume = rospy.Publisher('/PE/Drone/tank_volume', String, queue_size=10)
    pub_axisforces = rospy.Publisher('/drone_axisForces', Float32MultiArray, queue_size=10)
    pub_status = rospy.Publisher('/PE/Drone/drone_status', Float32MultiArray, queue_size=10) #Status structure: [rho, tiempito, Endpos, path_vel, route No.]
    pub_time = rospy.Publisher('PE/Drone/controller_time', Float32MultiArray, queue_size=10)
    pub_restart = rospy.Publisher('/PE/Drone/restart', Bool, queue_size=10)

    #Topic subscribers:
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
    rospy.Subscriber("/target_pose", Float32MultiArray, targetPose_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)

    #paths_file = input('Input paths file (no extention) >')
    paths_file = 'path_v1'
    scriptDir = os.path.dirname(__file__)
    paths_file = scriptDir +'/' + paths_file + '.txt'
    file = open(paths_file)

    delta_realTime = 0
    delta_simTime = 0
    lastDronePose = 0
    droneVel = 0
    restartTank = False

    avance = Float32MultiArray()
    avance_eu = Float32MultiArray()
    drone_status = Float32MultiArray()
    controller_time = Float32MultiArray()
    #axisForces = Float32MultiArray()
    tankVolume = String()
    
    tankVolume.data = 'B10L' #Volume of the water tank.

    while not rospy.is_shutdown():

        line = file.readline() #Reads first path

        #Go over each line of the paths file
        while line != '':
            tiempo = []
            ruta = []
            contador = 0
            
            linea = line.split('|')

            #Get each of the points in the path file
            ruta_list = linea[1].split(';')
            for i in range(len(ruta_list)):
                ruta.append(ruta_list[i])
            
            #Get associated times to reach each of the points
            tiempo_list = linea[2].split(',')
            for i in range(len(tiempo_list)):
                tiempo.append(float(tiempo_list[i]))

            #Go over each point of the trayectory
            for coord in ruta:
                if contador < len(tiempo): #Go through each of the times in the list.
                    print('Waiting for variableMass...')
                    #sys.stdout.write("\033[K") # Clear to the end of line
                    #sys.stdout.write("\033[F") # Cursor up one line

                    while tankMass == 0 or restartTank == True:
                        init = False 
                        restartTank = False
                        pub_tank_volume.publish(tankVolume)
                        pub_restart.publish(restartTank)
                        if tankMass != 0:
                            init = True
                            break

                    #init = True

                    #If tank mass is already calculated, we can start.
                    if init == True:
                        tiempito = tiempo[contador]
                        
                        coord_aux = coord.split(',')
                        #print('coord_aux:   ', coord_aux)
                        if coord_aux[0] != '':
                            coord_x = coord_aux[0]
                            coord_y = coord_aux[1]
                            coord_z = coord_aux[2]
                            #endPos_theta = np.arctan2(float(coord_y), float(coord_x))
                            endPos = [float(coord_x), float(coord_y), float(coord_z) ] # [X. Y, Z]

                            deltaX = endPos[0] - posTarget_x #Distance error X-axis [particle]
                            deltaY = endPos[1] - posTarget_y #Distance error Y-axis [particle]

                            rho = np.sqrt(deltaX**2 + deltaY**2)

                            path_vel = rho/tiempito

                            contador = contador + 1

                            #Update simulation and real time.
                            simTime_actual = simTime
                            realTime_actual = realTime

                            delta_simTime = simTime_actual - simTime_anterior
                            delta_realTime = realTime_actual - realTime_anterior

                            simTime_anterior = simTime_actual
                            sim_anterior2 = simTime_actual #helps checking if 1 second has passed in simulation.
                            realTime_anterior = realTime_actual

                            #Target (particle) conestant velocity.
                            v_x = deltaX/tiempito    
                            v_y = deltaY/tiempito

                            #Distance to target (particle)
                            target_rho = np.sqrt((posTarget_x-pos_x)**2 + (posTarget_y-pos_y)**2)

                            #While we reach the point (with certain error), keep moving.
                            while rho > 0.1:

                                actualDronePose = pos_x

                                target_rho = np.sqrt((posTarget_x-pos_x)**2 + (posTarget_y-pos_y)**2)

                                #If drone went out of control and lost the target. Restart.
                                if target_rho > 3:
                                    print('++++++++++++++++++++++++ Target too far... Restarting simulation')
                                    restartTank = True
                                    pub_restart.publish(restartTank)
                                    time.sleep(8)
                                    restartTank = False
                                    pub_restart.publish(restartTank)
                                    break

                                #pub_tank_volume.publish(tankVolume) #Select tank volume.
                                simTime_actual = simTime
                                realTime_actual = realTime

                                delta_simTime = simTime_actual - simTime_anterior
                                delta_realTime = realTime_actual - realTime_anterior

                                deltaX = endPos[0] - posTarget_x #Distance error X-axis [particle]
                                deltaY = endPos[1] - posTarget_y #Distance error Y-axis [particle]

                                rho = np.sqrt(deltaX**2 + deltaY**2)

                                paso_x = v_x #+ vel_adjust_x
                                paso_y = v_y #+ vel_adjust_y

                                missing_points = len(ruta)-contador #How many points missing till finishing path

                                #Only when 1 second has passed in simulation, publish the new position.
                                #Simulation con go faster than real time and still behave as supposed.
                                if simTime_actual - sim_anterior2 >= 1:

                                    droneVel = actualDronePose - lastDronePose
                                    lastDronePose = actualDronePose
                                    
                                    sim_anterior2 = simTime_actual

                                    avance.data = [posTarget_x+paso_x, posTarget_y+paso_y, endPos[2]]
                                    avance_eu.data = [ang_x, ang_y, ang_z]
                                    drone_status.data = [rho, tiempito, endPos[0], endPos[1], endPos[2], path_vel, float(linea[0]), missing_points, droneVel] #Estructure : [rho, pathTime, EndPos, path_vel, route No., missing_points, dronevel]
                                    controller_time.data = [delta_realTime, delta_simTime]

                                    pub_pose.publish(avance)
                                    pub_euler.publish(avance_eu)
                                    #pub_axisforces.publish(axisForces)
                                    pub_status.publish(drone_status)
                                    pub_time.publish(controller_time)
                                    pub_tank_volume.publish(tankVolume)

                                print('Target_RHO: ' + str(round(target_rho,3)) )
                                sys.stdout.write("\033[K") # Clear to the end of line
                                sys.stdout.write("\033[F") # Cursor up one line

                            delta_simTime = simTime_actual - simTime_anterior
                            delta_realTime = realTime_actual - realTime_anterior
            
            #Wait till drone reaches particle before starting new path...
            
            while target_rho > 0.08:
                target_rho = np.sqrt((posTarget_x-pos_x)**2 + (posTarget_y-pos_y)**2)
                print('Wait till drone reaches particle before starting new path... ', round(target_rho,3))
                sys.stdout.write("\033[K") # Clear to the end of line
                sys.stdout.write("\033[F") # Cursor up one line
                #restartTank = True
                #pub_restart.publish(restartTank)
                
            restartTank = True
            pub_restart.publish(restartTank)

            print('--------------------------WAIT 10 SECONDS--------------------')
            time.sleep(10)
            
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