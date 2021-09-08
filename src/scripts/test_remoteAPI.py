#!/usr/bin/env python3
import vrep
import numpy as np

#Codigo que pureba alguna funciones del remote API, conecta con el drone, da posicion, intenta moverlo
#funciona con la escena simple_test.ttt
#simRemoteApi.start(19999)

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

#Cargar escena
#_ = vrep.simxLoadScene(clientID, r'/home/salo/tesis_ws/src/drones_pkg/src/Scenes/Quadcopter Terrain 1.ttt',0,vrep.simx_opmode_blocking)

#Empezar simulacion
#_ = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

returnCode,handle=vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_blocking)
Quadricopter_target = handle
#print(Quadricopter_target)

#Obtenemos la posicion del dron
returnCode,pos=vrep.simxGetObjectPosition(clientID, Quadricopter_target, -1, vrep.simx_opmode_blocking)
print('Pose del robot: ',pos)

#Movemos el robot
endpos = [1, 1, 1]
returnCodeMOV = vrep.simxSetObjectPosition(clientID, Quadricopter_target,-1, endpos, vrep.simx_opmode_blocking)

