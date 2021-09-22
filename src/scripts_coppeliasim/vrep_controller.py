#! /usr/bin/env python
import rospy
import sys
import numpy as np
import sim
from std_msgs.msg import Float32MultiArray, Float32

#intento de no usar el target obj de la simulacion si no coordenadas enviadas por ros
#funciona con la escena ros_test_newLUA.ttt o ros_torque_newLUA.ttt

pos_x, pos_y, pos_z, theta, deltaX, deltaY = 0, 0, 0, 0, 0, 0

ang_x, ang_y, ang_z = 0,0,0

rho = 0
K_rho = 0.10
K_alpha = 0.4

#Tiempo en el que quiero recorrer cada trayectoria en segundos [s]
tiempo = [10, 20, 30, 40, 50, 11, 21, 31, 41, 51, 61, 71]

#Puntos dentro de la ruta en metros [m]#tiempo = [100, 150, 200, 300, 100]
#ruta = np.array([[-4,-4], [-4, 4], [-2, 4], [-2, -4], [0,-4], [0,4], [2,4], [2, 0], [3, 0], [3, 4], [4,4], [-4,-4]])
ruta = [[-4,-4,1], [4, -4,1], [1, 1,1], [-2, -4,1], [0,-4,1], [0,4,1], [2,4,1], [2, 0,1], [3, 0,1], [3, 4,1], [4,4,1], [-4,-4,1]]


simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0
force = []

connect = False

print('Program started')
sim.simxFinish(-1) #close all opened connections, just in case

def connect(port):
    # Establece la conexion a VREP
    # port debe coincidir con el puerto de conexion en VREP
    # retorna el numero de cliente o -1 si no puede establecer conexion
    global connect
    sim.simxFinish(-1) # just in case, close all opened connections\n",
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: 
        print("Conectado a: ", port)
        connect = True
    else: 
        connect = False
        print("No se pudo conectar")
    return clientID

#Conectarse al servidor de VREP
clientID = connect(19999)

returnCode,handle=sim.simxGetObjectHandle(clientID,'Quadricopter_base',sim.simx_opmode_blocking)
Quadricopter_base = handle
print('Quadricopter Target handle: ', Quadricopter_base)

#Obtenemos la posicion del dron
def get_position():
    global pos_x, pos_y, pos_z
    _,pos=sim.simxGetObjectPosition(clientID, Quadricopter_base, -1, sim.simx_opmode_blocking)
    pos_x = pos[0]
    pos_y = pos[1]
    pos_z = pos[2]
    return pos_x, pos_y, pos_z

#Obtenemos la orientacion del dron
def get_orientation():
    global ang_x, ang_y, ang_z
    _, orientation = sim.simxGetObjectOrientation(clientID,Quadricopter_base,-1,sim.simx_opmode_blocking)
    ang_x = orientation[0]
    ang_y = orientation[1]
    ang_z = orientation[2]
    return ang_x, ang_y, ang_z

#Obtenemos el tiempo de simulacion
def simTime_callback(msg):
    global simTime
    simTime = msg.data

#Obtenemos tiempo real
def realTime_callback(msg):
    global realTime
    realTime = msg.data

def force_callback(msg):
    global force
    force = msg.data

def main_control():
    global pos_x, pos_y, pos_z, theta, deltaX, deltaY, K_rho, K_alpha, tiempo, ruta, ang_x, ang_y, ang_z, force
    global simTime_actual, realTime_actual, simTime_anterior, realTime_anterior, realTime, simTime, connect

    print('Starting control node...')
    print(' ')
    rospy.init_node('control', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    pub_pose = rospy.Publisher('drone_nextPose', Float32MultiArray, queue_size=10)
    pub_euler = rospy.Publisher('drone_nextEuler', Float32MultiArray, queue_size=10)

    #Subscripcion a nodos
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/force", Float32MultiArray, force_callback, tcp_nodelay=True)

    contador = 0
    v_x = 0
    v_y = 0
    delta_realTime = 0
    delta_simTime = 0

    avance = Float32MultiArray()
    avance_eu = Float32MultiArray()

    while not rospy.is_shutdown() and connect == True:
        #Recorremos cada punto en la ruta
        for coord in ruta:
            if contador < len(tiempo): #Para recorrer completamente la lista de tiempos
                    
                tiempo_path = tiempo[contador]*100/15.25
                coord_x = coord[0]
                coord_y = coord[1]
                endPos = [float(coord_x), float(coord_y), float(1) ] # [X. Y, Z]

                pos_x, pos_y, pos_z = get_position()
                ang_x, ang_y, ang_z = get_orientation()

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

                #if delta_simTime < tiempo_path:
                delta_simTime = simTime_actual - simTime_anterior
                delta_realTime = realTime_actual - realTime_anterior

                simTime_anterior = simTime_actual
                realTime_anterior = realTime_actual
                
                #mientras no lleguemos, siga avanzando
                while rho > 0.09:
                    simTime_actual = simTime
                    realTime_actual = realTime

                    #delta_simTime = simTime_actual - simTime_anterior
                    #delta_realTime = realTime_actual - realTime_anterior

                    print(str(round(force[1],3)) +  ' | RHO: ' + str(round(rho,3)) +' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) )
                    #print('realTime: ' + str(round(delta_realTime,4)) +' simTime: ' + str(round(delta_simTime,4)) + ' Time: ' + str(round(tiempo[contador-2],3)) + ' | RHO: ' + str(round(rho,3)) )# +' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) # +  ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))
                    sys.stdout.write("\033[K") # Clear to the end of line
                    sys.stdout.write("\033[F") # Cursor up one line
                    #time.sleep(1)

                    #Actualizamos posiciones
                    pos_x, pos_y, pos_z = get_position()
                    ang_x, ang_y, ang_z = get_orientation()

                    deltaX = endPos[0] - pos_x #distancias
                    deltaY = endPos[1] - pos_y

                    rho = np.sqrt(deltaX**2 + deltaY**2)
                    
                    paso_x = v_x
                    paso_y = v_y
                    #avance_2 = [pos_x+paso_x, pos_y+paso_y, 0.5]
                    avance.data = [pos_x+paso_x, pos_y+paso_y, endPos[2]]
                    avance_eu.data = [ang_x, ang_y, ang_z]

                    pub_pose.publish(avance)
                    pub_euler.publish(avance_eu)
                    #_ = sim.simxSetObjectPosition(clientID, Quadricopter_base,-1, avance_2, sim.simx_opmode_blocking)


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