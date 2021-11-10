#! /usr/bin/env python3
import rospy
import sys
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, String

#Controla el dron en cualquier direccion y se guia por la particula de CoppeliaSim
#Funciona muy bien con los tiempos puestos
#funciona con la escena tracking_control.ttt

pos_x, pos_y, pos_z, theta, deltaX, deltaY, posTarget_x,posTarget_y,posTarget_z = 0, 0, 0, 0, 0, 0, 0, 0, 0
ang_x, ang_y, ang_z = 0,0,0

masa = 26
rho = 0

#Outer Loop Gains - Desired roll and pitch, desired path time
kp_r, kp_p, kp_t, kd_t = 0.5, 0.5, 2, 0

#Inner Loop Gains - Thrust, Roll, Pitch, Yaw Control
pParam, iParam, dParam, vParam = 32,0,0.005,-2
pRoll, iRoll, dRoll = 2,0,0.005
pPitch, iPitch, dPitch = 64,0,0.005
pYaw, iYaw, dYaw = 128,0,0.05

#Tiempo en el que quiero recorrer cada trayectoria en segundos [s]
tiempo = [80, 40, 150, 50]

#Puntos dentro de la ruta en metros [m]
ruta = [[0,0,1], [-3,0,1], [-3,3,1], [3,3,1], [0,4,1]]


simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0
tankMass = 0
init = False
feedback_vel, feedback_vel_x, feedback_vel_y = 0,0,0

print('Program started')

#Obtenemos la posicion del dron
def dronePose_callback(msg):
    global pos_x, pos_y, pos_z
    #_,pos=sim.simxGetObjectPosition(clientID, Quadricopter_base, -1, sim.simx_opmode_blocking)
    pos_x = msg.data[0]
    pos_y = msg.data[1]
    pos_z = msg.data[2]
    return pos_x, pos_y, pos_z

def targetPose_callback(msg):
    global posTarget_x, posTarget_y, posTarget_z
    posTarget_x = msg.data[0]
    posTarget_y = msg.data[1]
    posTarget_z = msg.data[2]


#Obtenemos la orientacion del dron
def droneOrientation_callback(msg):
    global ang_x, ang_y, ang_z
    #_, orientation = sim.simxGetObjectOrientation(clientID,Quadricopter_base,-1,sim.simx_opmode_blocking)
    ang_x = msg.data[0]
    ang_y = msg.data[1]
    ang_z = msg.data[2]
    return ang_x, ang_y, ang_z

#Obtenemos el tiempo de simulacion
def simTime_callback(msg):
    global simTime
    simTime = msg.data

#Obtenemos tiempo real
def realTime_callback(msg):
    global realTime
    realTime = msg.data

#Obtenemos el peso del tanque
def tankMass_callback(msg):
    global tankMass
    tankMass = msg.data

def droneVelocity_callback(msg):
    global feedback_vel, feedback_vel_x, feedback_vel_y
    feedback_vel_x = msg.data[0]
    feedback_vel_y = msg.data[1]

    feedback_vel = np.sqrt(feedback_vel_x**2 + feedback_vel_y**2)

#Funcion principal donde corren todos los controladores
def main_control():
    global pos_x, pos_y, pos_z, theta, deltaX, deltaY, tiempo, ruta, ang_x, ang_y, ang_z, posTarget_x,posTarget_y,posTarget_z
    global simTime_actual, realTime_actual, simTime_anterior, realTime_anterior, realTime, simTime, tankMass, init

    print('Starting control node...')
    print(' ')
    rospy.init_node('Control_Node', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    #Publicacion de topicos
    pub_pose = rospy.Publisher('/drone_nextPose', Float32MultiArray, queue_size=10)
    pub_euler = rospy.Publisher('/drone_nextEuler', Float32MultiArray, queue_size=10)
    pub_tank_volume = rospy.Publisher('/PE/Drone/tank_volume', String, queue_size=10)
    pub_axisforces = rospy.Publisher('/drone_axisForces', Float32MultiArray, queue_size=10)

    #Estructura : [rho, tiempito, Endpos]
    pub_status = rospy.Publisher('PE/Drone/drone_status', Float32MultiArray, queue_size=10)
    pub_time = rospy.Publisher('PE/Drone/controller_time', Float32MultiArray, queue_size=10)

    #Subscripcion a topicos
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
    rospy.Subscriber("/target_pose", Float32MultiArray, targetPose_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)
    rospy.Subscriber("/velocity", Float32MultiArray, droneVelocity_callback, tcp_nodelay=True)
    #rospy.Subscriber("PE/Drone/init_flag", Bool, init_callback, tcp_nodelay=True)

    contador = 0
    desired_vel_x, desired_vel_y = 0,0
    delta_realTime = 0
    delta_simTime = 0

    cumul, cumulRoll, cumulPitch, cumulYaw, lastE, lastERoll, lastEPitch, lastEYaw, lastEVel_x, lastEVel_y = 0,0,0,0,0,0,0,0,0,0
    thrust, roll, pitch, yaw = 0,0,0,0

    avance = Float32MultiArray()
    avance_eu = Float32MultiArray()
    drone_status = Float32MultiArray()
    controller_time = Float32MultiArray()
    axisForces = Float32MultiArray()

    while not rospy.is_shutdown():
        #Recorremos cada punto en la ruta
        
        for coord in ruta:
            if contador < len(tiempo): #Para recorrer completamente la lista de tiempos
                print('Waiting for variableMass...')
                sys.stdout.write("\033[K") # Clear to the end of line
                sys.stdout.write("\033[F") # Cursor up one line

                pub_tank_volume.publish('B15L')
                
                if tankMass != 0:
                    init = True 
                else: 
                    init = False
                init = True
                if init == True:
                    tiempito = tiempo[contador]

                    coord_x = coord[0]
                    coord_y = coord[1]
                    coord_z = coord[2]
                    endPos_theta = np.arctan2(float(coord_y), float(coord_x))
                    endPos = [float(coord_x), float(coord_y), float(coord_z) ] # [X. Y, Z]

                    deltaX = endPos[0] - posTarget_x #Distancia que me falta en X [particula]
                    deltaY = endPos[1] - posTarget_y #Distancia que me falta en Y [particula]
                    deltaZ = endPos[2] - pos_z

                    rho = np.sqrt(deltaX**2 + deltaY**2)
                    init_rho = rho

                    #ponemos al dron a volar antes de iniciar el recorrido
                    while deltaZ > 0.2:

                        deltaX = endPos[0] - posTarget_x #Distancia que me falta en X
                        deltaY = endPos[1] - posTarget_y #Distancia que me falta en Y
                        deltaZ = endPos[2] - pos_z

                        rho = np.sqrt(deltaX**2 + deltaY**2)

                        # -- Vertical control:
                        cumul=cumul+deltaZ
                        thrust = masa * (pParam*deltaZ + iParam*cumul + dParam*(deltaZ-lastE))
                        lastE=deltaZ

                        axisForces.data = [0, 0, 0, 0]
                        drone_status.data = [rho, tiempito, endPos[0], endPos[1], endPos[2]]

                        pub_axisforces.publish(axisForces)
                        pub_status.publish(drone_status)

                        print('Ajustando altura: '+str(thrust) + '  Delta Z: ' + str(deltaZ))
                        sys.stdout.write("\033[K") # Clear to the end of line
                        sys.stdout.write("\033[F") # Cursor up one line

                    contador = contador + 1

                    #Calculamos tiempo en simulacion y tiempo real
                    simTime_actual = simTime
                    realTime_actual = realTime

                    #if delta_simTime < tiempo_path:
                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior

                    simTime_anterior = simTime_actual
                    sim_anterior2 = simTime_actual
                    realTime_anterior = realTime_actual

                    #Calculamos velocidad constante dependiendo de la direccion
                    path_vel = init_rho/tiempito

                    #Calculamos velocidad constante a la que debería ir el target
                    v_x = deltaX/tiempito    
                    v_y = deltaY/tiempito

                    #mientras no lleguemos, siga avanzando
                    while rho > 0.2:
                        pub_tank_volume.publish('B15L')
                        simTime_actual = simTime
                        realTime_actual = realTime

                        delta_simTime = simTime_actual - simTime_anterior
                        delta_realTime = realTime_actual - realTime_anterior

                        deltaX = endPos[0] - posTarget_x #Distancia que me falta en X
                        deltaY = endPos[1] - posTarget_y #Distancia que me falta en Y
                        deltaZ = endPos[2] - pos_z

                        rho = np.sqrt(deltaX**2 + deltaY**2)

                        #Push (empuje) control:
                        vel_adjust_x = kp_t*(path_vel-feedback_vel_x) + kd_t*((path_vel-feedback_vel_x)-lastEVel_x)
                        lastEVel_x = (path_vel-feedback_vel_x)

                        vel_adjust_y = kp_t*(path_vel-feedback_vel_y) + kd_t*((path_vel-feedback_vel_y)-lastEVel_y)
                        lastEVel_y = (path_vel-feedback_vel_y)

                        # -- Vertical control:
                        cumul=cumul+deltaZ
                        thrust = masa * (pParam*deltaZ + iParam*cumul + dParam*(deltaZ-lastE))
                        lastE=deltaZ

                        paso_x = v_x #+ vel_adjust_x
                        paso_y = v_y #+ vel_adjust_y

                        #La velocidad esta en metros por segundo, solo hasta que en el tiempo de simulacion hayan pasado 1 segundo se publica.
                        if simTime_actual - sim_anterior2 >= 1:
                            sim_anterior2 = simTime_actual
                            avance.data = [posTarget_x+paso_x, posTarget_y+paso_y, endPos[2]]
                            avance_eu.data = [ang_x, ang_y, ang_z]
                            axisForces.data = [0, 0, 0, 0]
                            drone_status.data = [rho, tiempito, endPos[0], endPos[1], endPos[2]]
                            controller_time.data = [delta_realTime, delta_simTime]

                            pub_pose.publish(avance)
                            pub_euler.publish(avance_eu)
                            pub_axisforces.publish(axisForces)
                            pub_status.publish(drone_status)
                            pub_time.publish(controller_time)

                        print('Delta_sim: ' + str(round(delta_simTime,3)) +  '  time_control: ' + str(round(simTime_actual - sim_anterior2,3)) )
                        sys.stdout.write("\033[K") # Clear to the end of line
                        sys.stdout.write("\033[F") # Cursor up one line

                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior

        print('----------------------------Termine la ruta--------------------------')
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line
        rate.sleep()

if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Nodo detenido')