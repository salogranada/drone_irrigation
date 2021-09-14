#!/usr/bin/env python
import sim
import numpy as np
import sys
import time
import rospy
from std_msgs.msg import Float32

#Codigo de control, con recorridos en el tiempo y recorrido uniforme.
#TIEMPOS REALES(?)
#funciona con la escena ros_test.ttt
#Funcionalidad Completa
#simRemoteApi.start(19999)

global pos_x, pos_y, pos_z, theta, deltaX, deltaY
global rho, ruta, hayRuta, tiempo
global K_alpha, K_rho
global simTime_anterior, realTime_anterior, realTime_actual, simTime_actual,realTime, simTime

pos_x, pos_y, pos_z, theta, deltaX, deltaY = 0, 0, 0, 0, 0, 0

rho = 0
K_rho = 0.10
K_alpha = 0.4
#Tiempo en el que quiero recorrer cada trayectoria en segundos [s]
#tiempo = [100, 150, 200, 300, 100]
tiempo = [10, 20, 30, 40, 50, 11, 201, 31, 41, 51, 61, 71]

#Puntos dentro de la ruta en metros [m]
#ruta = np.array([[-2,-2], [-2, 2], [2, 2], [2, -2], [-2,-2]])
ruta = np.array([[-4,-4], [-4, 4], [-2, 4], [-2, -4], [0,-4], [0,4], [2,4], [2, 0], [3, 0], [3, 4], [4,4], [-4,-4]])

simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0

print('Program started')
sim.simxFinish(-1) #close all opened connections, just in case

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
clientID = connect(19999)

returnCode,handle=sim.simxGetObjectHandle(clientID,'Quadricopter_target',sim.simx_opmode_blocking)
Quadricopter_target = handle
print('Quadricopter Target handle: ', Quadricopter_target)

#Obtenemos la posicion del dron
def get_position():
    global pos_x, pos_y, pos_z
    _,pos=sim.simxGetObjectPosition(clientID, Quadricopter_target, -1, sim.simx_opmode_blocking)
    pos_x = pos[0]
    pos_y = pos[1]
    pos_z = pos[2]
    return pos_x, pos_y, pos_z

#Obtenemos la orientacion del dron
def get_orientation():
    global theta
    _, orientation = sim.simxGetObjectOrientation(clientID,Quadricopter_target,-1,sim.simx_opmode_blocking)
    theta = orientation[2]
    return theta

#Movemos el robot a su posicion inicial
inicialpos = [-4, -4, 1]
returnCodeMOV = sim.simxSetObjectPosition(clientID, Quadricopter_target,-1, inicialpos, sim.simx_opmode_blocking)
print(returnCodeMOV)

#Obtenemos el tiempo de simulacion
def simTime_callback(msg):
    global simTime
    simTime = msg.data

#Obtenemos tiempo real
def realTime_callback(msg):
    global realTime
    realTime = msg.data

def main_control():
    global pos_x, pos_y, pos_z, theta, deltaX, deltaY, K_rho, K_alpha, tiempo, ruta
    global simTime_actual, realTime_actual, simTime_anterior, realTime_anterior, realTime, simTime

    print('Starting control node...')
    print(' ')
    rospy.init_node('control', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    #Subscripcion a nodos
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)

    contador = 0
    v_x = 0
    v_y = 0
    delta_realTime = 0
    delta_simTime = 0

    while not rospy.is_shutdown():
        #Recorremos cada punto en la ruta
        for coord in ruta:
            if contador < len(tiempo):
                    
                tiempo_path = tiempo[contador]*100/15.25
                coord_x = coord[0]
                coord_y = coord[1]
                endPos = [float(coord_x), float(coord_y), float(1) ] # [X. Y, Z]

                pos_x, pos_y, pos_z = get_position()
                theta = get_orientation()

                deltaX = endPos[0] - pos_x
                deltaY = endPos[1] - pos_y

                #Calculamos velocidad constante
                v_x = deltaX/tiempo_path    
                v_y = deltaY/tiempo_path

                rho = np.sqrt(deltaX**2 + deltaY**2)
                contador = contador + 1

                #Calculamos tiempo en simulacion y tiempo real
                simTime_actual = simTime
                realTime_actual = realTime

                if delta_simTime < tiempo_path:
                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior

                    simTime_anterior = simTime_actual
                    realTime_anterior = realTime_actual
                    
                    #mientras no lleguemos, siga avanzando
                    while rho > 0.06:
                        simTime_actual = simTime
                        realTime_actual = realTime

                        #delta_simTime = simTime_actual - simTime_anterior
                        #delta_realTime = realTime_actual - realTime_anterior

                        print('realTime: ' + str(round(delta_realTime,4)) +' simTime: ' + str(round(delta_simTime,4)) + ' Tiempo: ' + str(round(tiempo[contador-2],3)) + ' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | RHO: ' + str(round(rho,3)) )#+  ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))
                        sys.stdout.write("\033[K") # Clear to the end of line
                        sys.stdout.write("\033[F") # Cursor up one line
                        #time.sleep(1)

                        #Actualizamos posiciones
                        pos_x, pos_y, pos_z = get_position()
                        theta = get_orientation()

                        deltaX = endPos[0] - pos_x #distancias
                        deltaY = endPos[1] - pos_y

                        rho = np.sqrt(deltaX**2 + deltaY**2)
                        
                        paso_x = v_x
                        paso_y = v_y
                        avance = [pos_x+paso_x, pos_y+paso_y, 1]

                        _ = sim.simxSetObjectPosition(clientID, Quadricopter_target,-1, avance, sim.simx_opmode_blocking)


        print('----------------------------Termine la ruta-----------------------------------------')
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line
        rate.sleep()

#main_control()

if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Nodo detenido')