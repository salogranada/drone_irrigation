#! /usr/bin/env python3
import rospy
import sys
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, String

#Drone control guided by particle in simulation.
#Set the list of points in the trajectory and time the drone must reach each point.
#Coppeliasim Scene: tracking_control.ttt

#Author: Salomón Granada Ulloque
#Email: s.granada@uniandes.edu.co

pos_x, pos_y, pos_z, theta, deltaX, deltaY, posTarget_x,posTarget_y,posTarget_z = 0, 0, 0, 0, 0, 0, 0, 0, 0
ang_x, ang_y, ang_z = 0,0,0

#Drone mass
masa = 26
rho = 0

#Volume of the water tank.
tankVolume = 'B15L'

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

    #Publishing structure: [rho, tiempito, Endpos]
    pub_status = rospy.Publisher('/PE/Drone/drone_status', Float32MultiArray, queue_size=10)
    pub_time = rospy.Publisher('PE/Drone/controller_time', Float32MultiArray, queue_size=10)

    #Topic subscribers:
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
    rospy.Subscriber("/target_pose", Float32MultiArray, targetPose_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)
    #rospy.Subscriber("/velocity", Float32MultiArray, droneVelocity_callback, tcp_nodelay=True)
    #rospy.Subscriber("PE/Drone/init_flag", Bool, init_callback, tcp_nodelay=True)

    contador = 0
    delta_realTime = 0
    delta_simTime = 0
    lastDronePose = 0
    droneVel = 0

    avance = Float32MultiArray()
    avance_eu = Float32MultiArray()
    drone_status = Float32MultiArray()
    controller_time = Float32MultiArray()
    axisForces = Float32MultiArray()

    while not rospy.is_shutdown():

        #Go over each point of the trayectory
        for coord in ruta:
            if contador < len(tiempo): #Go through each of the times in the list.
                print('Waiting for variableMass...')
                sys.stdout.write("\033[K") # Clear to the end of line
                sys.stdout.write("\033[F") # Cursor up one line

                pub_tank_volume.publish(tankVolume) #Select tank volume.
                
                if tankMass != 0:
                    init = True 
                else: 
                    init = False
                init = True

                #If tank mass is already calculated, we can start.
                if init == True:
                    tiempito = tiempo[contador]

                    coord_x = coord[0]
                    coord_y = coord[1]
                    coord_z = coord[2]
                    endPos_theta = np.arctan2(float(coord_y), float(coord_x))
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

                    #While we reach the point (with certain error), keep moving.
                    while rho > 0.2:

                        actualDronePose = pos_x

                        pub_tank_volume.publish(tankVolume) #Select tank volume.
                        simTime_actual = simTime
                        realTime_actual = realTime

                        delta_simTime = simTime_actual - simTime_anterior
                        delta_realTime = realTime_actual - realTime_anterior

                        deltaX = endPos[0] - posTarget_x #Distance error X-axis [particle]
                        deltaY = endPos[1] - posTarget_y #Distance error Y-axis [particle]

                        rho = np.sqrt(deltaX**2 + deltaY**2)

                        paso_x = v_x #+ vel_adjust_x
                        paso_y = v_y #+ vel_adjust_y

                        #Only when 1 second has passed in simulation, publish the new position.
                        #Simulation con go faster than real time and still behave as supposed.
                        if simTime_actual - sim_anterior2 >= 1:

                            droneVel = actualDronePose - lastDronePose
                            lastDronePose = actualDronePose
                            
                            sim_anterior2 = simTime_actual

                            avance.data = [posTarget_x+paso_x, posTarget_y+paso_y, endPos[2]]
                            avance_eu.data = [ang_x, ang_y, ang_z]
                            drone_status.data = [rho, tiempito, endPos[0], endPos[1], endPos[2]]
                            controller_time.data = [delta_realTime, delta_simTime]

                            pub_pose.publish(avance)
                            pub_euler.publish(avance_eu)
                            pub_axisforces.publish(axisForces)
                            pub_status.publish(drone_status)
                            pub_time.publish(controller_time)

                        print('path_vel: ' + str(round(path_vel,4)) + '  droneVel: ' + str(round(droneVel,4)) + '   tiempito: ' + str(round(tiempito,3)) + '   Delta_sim: ' + str(round(delta_simTime,3)) )
                        sys.stdout.write("\033[K") # Clear to the end of line
                        sys.stdout.write("\033[F") # Cursor up one line

                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior

        print('----------------------------Finished Trayectory--------------------------')
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line
        rate.sleep()

if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Stoped node.')