#!/usr/bin/env python3
import vrep
import numpy as np
import sys
import time

#Codigo de control, con recorridos en el tiempo y recorrido uniforme.
#TIEMPOS REALES
#funciona con la escena simple_test.ttt
#simRemoteApi.start(19999)

global pos_x, pos_y, pos_z, theta, deltaX, deltaY
global rho, ruta, hayRuta, tiempo
global K_alpha, K_rho, K_beta

pos_x, pos_y, pos_z, theta, deltaX, deltaY = 0, 0, 0, 0, 0, 0

rho = 0
K_rho = 0.10
K_alpha = 0.4
#tiempo = [100, 150, 200, 300, 100]
tiempo = [10, 10, 5, 20, 10, 5, 10, 10, 5, 20, 10, 50]
#ruta = np.array([[-2,-2], [-2, 2], [2, 2], [2, -2], [-2,-2]])
ruta = np.array([[-4,-4], [-4, 4], [-2, 4], [-2, -4], [0,-4], [0,4], [2,4], [2, 0], [3, 0], [3, 4], [4,4], [-4,-4]])

print('Program started')
vrep.simxFinish(-1) #close all opened connections, just in case

def connect(port):
    # Establece la conexion a VREP
    # port debe coincidir con el puerto de conexión en VREP
    # retorna el número de cliente o -1 si no puede establecer conexión
    vrep.simxFinish(-1) # just in case, close all opened connections\n",
    clientID=vrep.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado a", port)
    else: print("No se pudo conectar")
    return clientID

#Conectarse al servidor de VREP
clientID = connect(19999)

returnCode,handle=vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_blocking)
Quadricopter_target = handle
print('Quadricopter Target handle: ', Quadricopter_target)

#Obtenemos la posicion del dron
def get_position():
    global pos_x, pos_y, pos_z
    _,pos=vrep.simxGetObjectPosition(clientID, Quadricopter_target, -1, vrep.simx_opmode_blocking)
    pos_x = pos[0]
    pos_y = pos[1]
    pos_z = pos[2]
    return pos_x, pos_y, pos_z

#Obtenemos la orientacion del dron
def get_orientation():
    global theta
    _, orientation = vrep.simxGetObjectOrientation(clientID,Quadricopter_target,-1,vrep.simx_opmode_blocking)
    theta = orientation[2]
    return theta

#Movemos el robot a su posicion inicial
inicialpos = [-4, -4, 1]
returnCodeMOV = vrep.simxSetObjectPosition(clientID, Quadricopter_target,-1, inicialpos, vrep.simx_opmode_blocking)
print(returnCodeMOV)

def main_control():
    global pos_x, pos_y, pos_z, theta, deltaX, deltaY, K_rho, K_alpha, tiempo, ruta
    contador = 0
    simTime_anterior = 0
    v_x = 0
    v_y = 0
    #Recorremos cada punto en la ruta
    for coord in ruta:
        time_actual = tiempo[contador]*100/15.25
        coord_x = coord[0]
        coord_y = coord[1]
        endPos = [float(coord_x), float(coord_y), float(1) ] # [X. Y, Z]

        pos_x, pos_y, pos_z = get_position()
        theta = get_orientation()

        deltaX = endPos[0] - pos_x
        deltaY = endPos[1] - pos_y

        #Calculamos velocidad constante
        v_x = deltaX/time_actual    
        v_y = deltaY/time_actual

        rho = np.sqrt(deltaX**2 + deltaY**2)
        contador = contador + 1

        #Calculamos tiempo en simulacion
        simTime_actual = vrep.simxGetLastCmdTime(clientID)
        delta_Time = simTime_actual - simTime_anterior
        simTime_anterior = simTime_actual

        #mientras no lleguemos, siga avanzando
        while rho > 0.06:
            
            print('simTime: ' + str(round(delta_Time/1000,4)) + ' Tiempo: ' + str(round(tiempo[contador-2],3)) + ' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | RHO: ' + str(round(rho,3)) )#+  ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))
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

            _ = vrep.simxSetObjectPosition(clientID, Quadricopter_target,-1, avance, vrep.simx_opmode_blocking)


    print('Termine la ruta')
    time.sleep(10000)

main_control()