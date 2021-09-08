#!/usr/bin/env python
import vrep
import numpy
import rospy
import time
from geometry_msgs.msg import Twist

# Establece la conexion a VREP
# port debe coincidir con el puerto de conexión en VREP
# retorna el número de cliente o -1 si no puede establecer conexión
def connect(port):
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado a", port)
    else: print("No se pudo conectar")
    return clientID

#Conectarse al servidor de VREP
clientID = connect(19999)

#Obtener el manejador del objeto.
returnCode,handle=vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_blocking)
Quadricopter_base = handle
print(Quadricopter_base)

#Obtener la posicion del objeto
returnCode,pos=vrep.simxGetObjectPosition(clientID, Quadricopter_base, -1, vrep.simx_opmode_blocking)
print(pos)


def main():
    print('Starting move node...')
    print(' ')
    rospy.init_node('move', anonymous=True) #Inicio nodo
    pub = rospy.Publisher('/quad_cmd_twist', Twist, queue_size=10)
    rate = rospy.Rate(10) #10hz

    vel_robot = Twist()
    while not rospy.is_shutdown():
        vel_robot.linear.x = 0
        vel_robot.linear.y = 0
        vel_robot.linear.z = 0
        vel_robot.angular.z = 0
        pub.publish(vel_robot)

        