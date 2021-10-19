#! /usr/bin/env python
import rospy
import sys
import numpy as np
import sim
from std_msgs.msg import Float32MultiArray, Float32, String, Bool

#Busca controlar desde este script el desplazamiento horizontal. Antes se manejaba desde VREP.
#funciona con la escena external_control.ttt

pos_x, pos_y, pos_z, theta, deltaX, deltaY = 0, 0, 0, 0, 0, 0

ang_x, ang_y, ang_z = 0,0,0

masa = 26
rho = 0
gamma = 0
K_rho = 0.20
K_gamma = 0.21 #Afecta Calculo del tiempo

pParam, iParam, dParam, vParam = 128,0,0,-2
pRoll, iRoll, dRoll = 2,0,0.005
pPitch, iPitch, dPitch = 128,0,0.005
pYaw, iYaw, dYaw = 8,0.001,0

#Tiempo en el que quiero recorrer cada trayectoria en segundos [s]
#tiempo = [10, 20, 30, 40, 50, 11, 21, 31, 41, 51, 61, 71]
tiempo = [20, 20, 20]

#Puntos dentro de la ruta en metros [m]#tiempo = [100, 150, 200, 300, 100]
#ruta = np.array([[-4,-4], [-4, 4], [-2, 4], [-2, -4], [0,-4], [0,4], [2,4], [2, 0], [3, 0], [3, 4], [4,4], [-4,-4]])
ruta = [[-4,-4,1], [-4, 0,1], [-4, -4,1], [-2, -4,1], [0,-4,2], [0,4,1], [2,4,1], [2, 0,1], [3, 0,1], [3, 4,1], [4,4,1], [-4,-4,1]]
#ruta = [[-4,-4,1], [-4, 4,1], [-4, -4,1]]

simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0
tankMass = 0
init = False

connect = False

print('Program started')
# sim.simxFinish(-1) #close all opened connections, just in case

# def connect(port):
#     # Establece la conexion a VREP
#     # port debe coincidir con el puerto de conexion en VREP
#     # retorna el numero de cliente o -1 si no puede establecer conexion
#     global connect
#     sim.simxFinish(-1) # just in case, close all opened connections\n",
#     clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
#     if clientID == 0: 
#         print("Conectado a: ", port)
#         connect = True
#     else: 
#         connect = False
#         print("No se pudo conectar")
#     return clientID

# #Conectarse al servidor de VREP
# clientID = connect(19999)

# returnCode,handle=sim.simxGetObjectHandle(clientID,'Quadcopter_base',sim.simx_opmode_blocking)
# Quadricopter_base = handle
# print('Quadricopter Target handle: ', Quadricopter_base)

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

#Obtenemos el tiempo de simulacion
def simTime_callback(msg):
    global simTime
    simTime = msg.data

#Obtenemos tiempo real
def realTime_callback(msg):
    global realTime
    realTime = msg.data

def tankMass_callback(msg):
    global tankMass
    tankMass = msg.data

def main_control():
    global pos_x, pos_y, pos_z, theta, deltaX, deltaY, K_rho, tiempo, ruta, ang_x, ang_y, ang_z, gamma
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

    #Estructura : [pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, tiempito]
    pub_status = rospy.Publisher('PE/Drone/drone_status', Float32MultiArray, queue_size=10)
    pub_time = rospy.Publisher('PE/Drone/controller_time', Float32MultiArray, queue_size=10)

    #Subscripcion a topicos
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/currentMass", Float32, tankMass_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_pose", Float32MultiArray, dronePose_callback, tcp_nodelay=True)
    rospy.Subscriber("/drone_orientation", Float32MultiArray, droneOrientation_callback, tcp_nodelay=True)
    #rospy.Subscriber("PE/Drone/init_flag", Bool, init_callback, tcp_nodelay=True)

    contador = 0
    v_x = 0
    v_y = 0
    v_z = 0
    delta_realTime = 0
    delta_simTime = 0

    cumul, cumulRoll, cumulPitch, cumulYaw, lastE, lastERoll, lastEPitch = 0,0,0,0,0,0,0
    thrust, roll, pitch, yaw = 0,0,0,0

    avance = Float32MultiArray()
    avance_eu = Float32MultiArray()
    drone_status = Float32MultiArray()
    controller_time = Float32MultiArray()
    axisForces = Float32MultiArray()

    initPitch = ang_y

    while not rospy.is_shutdown():
        #Recorremos cada punto en la ruta
        
        for coord in ruta:
            if contador < len(tiempo): #Para recorrer completamente la lista de tiempos
                print('Waiting for variableMass...')
                sys.stdout.write("\033[K") # Clear to the end of line
                sys.stdout.write("\033[F") # Cursor up one line

                pub_tank_volume.publish('B30L')
                
                if tankMass != 0:
                    init = True 
                else: 
                    init = False
                init = True
                if init == True:
                    tiempito = tiempo[contador]
                    #tiempo_path = tiempo[contador]
                    #tiempo_path = (6.64417*tiempo[contador]) - 0.49887
                    #tiempo_path = 5.433045*pow(tiempo[contador],1.08946378)
                    tiempo_path = (0.0765637*pow(tiempito,3) - 1.6490187*pow(tiempito,2) + 16.23514*tiempito - 13.171232933)*K_gamma
                    #print('tiempo path: ', tiempo_path)
                    coord_x = coord[0]
                    coord_y = coord[1]
                    coord_z = coord[2]
                    endPos_theta = np.arctan2(float(coord_y), float(coord_x))
                    endPos = [float(coord_x), float(coord_y), float(coord_z) ] # [X. Y, Z]

                    deltaX = endPos[0] - pos_x #Distancia que me falta en X
                    deltaY = endPos[1] - pos_y #Distancia que me falta en Y

                    rho = np.sqrt(deltaX**2 + deltaY**2)
                    alpha = -ang_z + np.arctan2(deltaY, deltaX)
                    contador = contador + 1

                    #Calculamos tiempo en simulacion y tiempo real
                    simTime_actual = simTime
                    realTime_actual = realTime

                    #if delta_simTime < tiempo_path:
                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior

                    simTime_anterior = simTime_actual
                    realTime_anterior = realTime_actual

                    #Calculamos velocidad constante dependiendo de la direccion
                    v_x = deltaX/tiempo_path    
                    v_y = deltaY/tiempo_path

                    # #Cuadramos Orientacion
                    # while alpha > 0.06:
                    #     deltaX = endPos[0] - pos_x #Distancia que me falta en X
                    #     deltaY = endPos[1] - pos_y #Distancia que me falta en Y
                    #     deltaZ = endPos[2] - pos_z

                    #     deltaRoll = 0 - ang_x
                    #     deltaPitch = 0 - ang_y
                    #     deltaYaw = endPos_theta - ang_z #Angulo al que mira el dron

                    #     # -- Vertical control:
                    #     cumul=cumul+deltaZ
                    #     thrust = masa * (pParam*deltaZ + iParam*cumul + dParam*(deltaZ-lastE))
                    #     lastE=deltaZ

                    #     # -- Orientation Control
                    #     #pYaw = 32 + 0.3 * np.exp(-alpha)
                    #     cumulYaw=cumulYaw+deltaYaw
                    #     alpha = -ang_z + np.arctan2(deltaY, deltaX)
                    #     yaw = masa * (pYaw*deltaYaw + iYaw*cumulYaw)

                    #     axisForces.data = [thrust, 0, 0, yaw]
                    #     drone_status.data = [pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, tiempito, endPos[0], endPos[1], endPos[2]]
                    #     controller_time.data = [delta_realTime, delta_simTime]

                    #     pub_axisforces.publish(axisForces)
                    #     pub_pose.publish(avance)
                    #     pub_euler.publish(avance_eu)
                    #     pub_status.publish(drone_status)
                    #     pub_time.publish(controller_time)

                    #     print('Alpha: ' + str(round(alpha,3)) + '  Yaw: ' + str(round(yaw,4)) + '  Thrust: ' + str(thrust))

                    #mientras no lleguemos, siga avanzando
                    while rho > 0.3:
                        pub_tank_volume.publish('B30L')
                        simTime_actual = simTime
                        realTime_actual = realTime

                        delta_simTime = simTime_actual - simTime_anterior
                        delta_realTime = realTime_actual - realTime_anterior

                        deltaX = endPos[0] - pos_x #Distancia que me falta en X
                        deltaY = endPos[1] - pos_y #Distancia que me falta en Y
                        deltaZ = endPos[2] - pos_z

                        deltaRoll = 0 - ang_x
                        deltaPitch = initPitch - ang_y
                        deltaYaw = endPos_theta - ang_z #Angulo al que mira el dron

                        rho = np.sqrt(deltaX**2 + deltaY**2)

                        # -- Vertical control:
                        cumul=cumul+deltaZ
                        thrust = masa * (pParam*deltaZ + iParam*cumul + dParam*(deltaZ-lastE))
                        lastE=deltaZ

                        # --Horizontal Control
                        cumulRoll=cumulRoll+deltaRoll
                        roll = (pRoll*deltaRoll + iRoll*cumulRoll + dRoll*(deltaRoll-lastERoll))*-1
                        lastERoll = deltaRoll

                        cumulPitch=cumulPitch+deltaPitch
                        pitch = (pPitch*deltaPitch + iPitch*cumulPitch + dPitch*(deltaPitch-lastEPitch))*-1
                        lastEPitch = deltaPitch

                        axisForces.data = [thrust, roll, pitch, yaw]
                        
                        drone_status.data = [pos_x, pos_y, pos_z, ang_x, ang_y, ang_z, rho, tiempito, endPos[0], endPos[1], endPos[2]]
                        controller_time.data = [delta_realTime, delta_simTime]

                        pub_axisforces.publish(axisForces)
                        pub_pose.publish(avance)
                        pub_euler.publish(avance_eu)
                        pub_status.publish(drone_status)
                        pub_time.publish(controller_time)

                        print(str(initPitch) +' delta_Pitch: ' + str(round(deltaPitch,3)) +' Pitch: ' + str(round(pitch,3)) + ' delta_Z: ' + str(round(deltaZ,3)) + '  Thrust: ' + str(round(thrust,3)))
                        #sys.stdout.write("\033[K") # Clear to the end of line
                        #sys.stdout.write("\033[F") # Cursor up one line

                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior


        #print('----------------------------Termine la ruta--------------------------')
        #sys.stdout.write("\033[K") # Clear to the end of line
        #sys.stdout.write("\033[F") # Cursor up one line
        rate.sleep()

if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Nodo detenido')