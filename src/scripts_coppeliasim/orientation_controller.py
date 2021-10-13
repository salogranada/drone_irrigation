#! /usr/bin/env python
import rospy
import sys
import numpy as np
import sim
from std_msgs.msg import Float32MultiArray, Float32, String, Bool

#Intento de ajustar la orientacion del dron
#el control para la altura se supone que se hace en vrep
#funciona con la escena ros_drone_integration.ttt

#Tiempo en el que quiero recorrer cada trayectoria en segundos [s]
#tiempo = [10, 20, 30, 40, 50, 11, 21, 31, 41, 51, 61, 71]
tiempo = [20, 20]

#Puntos dentro de la ruta en metros [m]
ruta = [[-4, 4,1], [1, 4,1], [-2, -4,1], [0,-4,1], [0,4,1], [2,4,1], [2, 0,1], [3, 0,1], [3, 4,1], [4,4,1], [-4,-4,1]]

pos_x, pos_y, pos_z, deltaX, deltaY = 0, 0, 0, 0, 0
ang_x, ang_y, ang_z = 0,0,0

rho = 0
gamma = 0
K_rho = 0.20
K_alpha = 0.4
K_gamma = 0.21 #Afecta Calculo del tiempo

simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0
simTime = 0
realTime = 0
force = []
velocity = 0
init = False

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

returnCode,handle=sim.simxGetObjectHandle(clientID,'Quadcopter_base',sim.simx_opmode_blocking)
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

def velocity_callback(msg):
    global velocity
    velocity = msg.data

def init_callback(msg):
    global init
    init = msg.data

def main_control():
    global pos_x, pos_y, pos_z, deltaX, deltaY, K_rho, K_alpha, tiempo, ruta, ang_x, ang_y, ang_z, force, gamma
    global simTime_actual, realTime_actual, simTime_anterior, realTime_anterior, realTime, simTime, connect, velocity, init

    print('Starting control node...')
    print(' ')
    rospy.init_node('control', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    #Publicacion de topicos
    pub_pose = rospy.Publisher('drone_nextPose', Float32MultiArray, queue_size=10)
    pub_euler = rospy.Publisher('drone_nextEuler', Float32MultiArray, queue_size=10)
    pub_irrigation_flag = rospy.Publisher('/irrigation_flag', String, queue_size=10)

    #Subscripcion a topicos
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/force", Float32MultiArray, force_callback, tcp_nodelay=True)
    rospy.Subscriber("/velocity", Float32, velocity_callback, tcp_nodelay=True)
    rospy.Subscriber("init_flag", Bool, init_callback, tcp_nodelay=True)

    contador = 0
    v_x = 0
    v_y = 0
    delta_realTime = 0
    delta_simTime = 0

    avance = Float32MultiArray()
    avance_eu = Float32MultiArray()
    print('Estamos conectados: ', connect)
    while not rospy.is_shutdown() and connect == True and Quadricopter_base != 0:

        #Recorremos cada punto en la ruta
        for coord in ruta:
            if contador < len(tiempo): #Para recorrer completamente la lista de tiempos
                print('Waiting for variableMass...')
                sys.stdout.write("\033[K") # Clear to the end of line
                sys.stdout.write("\033[F") # Cursor up one line

                init = True #Quitar cuando se quiera probar con VariableMass

                if init == True: #Indica si la masa variable ya fue calculada y se puede iniciar el movimiento.
                    
                    coord_x = coord[0]
                    coord_y = coord[1]
                    coord_z = coord[2]
                    endPos = [float(coord_x), float(coord_y), float(coord_z) ] # [X. Y, Z]

                    pos_x, pos_y, pos_z = get_position()
                    ang_x, ang_y, ang_z = get_orientation()

                    deltaX = endPos[0] - pos_x
                    deltaY = endPos[1] - pos_y

                    rho = np.sqrt(deltaX**2 + deltaY**2)
                    alpha = -ang_z + np.arctan2(deltaY, deltaX)

                    #Ajustamos el angulo (hacia donde mira el dron)
                    while abs(alpha) > 0.05 and connect == True and Quadricopter_base != 0:
					
                        print('ALFA: ' + str(round(alpha,3)) + ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(ang_z,3)))
                        #sys.stdout.write("\033[K") # Clear to the end of line
                        #sys.stdout.write("\033[F") # Cursor up one line

                        #Actualizamos posiciones
                        pos_x, pos_y, pos_z = get_position()
                        ang_x, ang_y, ang_z = get_orientation()
                       
                        deltaX = endPos[0] - pos_x
                        deltaY = endPos[1] - pos_y

                        alpha = -ang_z + np.arctan2(deltaY, deltaX)

                        K_alpha = 0.4 + 0.3 * np.exp(-alpha)

                        w_vel = K_alpha*alpha

                        avance_eu.data = [ang_x, ang_y, ang_z+w_vel]

                        pub_euler.publish(avance_eu)

                    #El tiempo solo empieza a correr cuando ya me voy a mover en X y Y
                    
                    tiempito = tiempo[contador]
                    #tiempo_path = tiempo[contador]
                    #tiempo_path = (6.64417*tiempo[contador]) - 0.49887
                    #tiempo_path = 5.433045*pow(tiempo[contador],1.08946378)
                    tiempo_path = (0.0765637*pow(tiempito,3) - 1.6490187*pow(tiempito,2) + 16.23514*tiempito - 13.171232933)*K_gamma
                    #print(tiempo_path)

                    #Calculamos velocidad constante dependiendo de la direccion
                    v_x = deltaX/tiempo_path    
                    v_y = deltaY/tiempo_path
                    
                    #Actualizamos tiempo en simulacion y tiempo real
                    simTime_actual = simTime
                    realTime_actual = realTime

                    #if delta_simTime < tiempo_path:
                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior

                    simTime_anterior = simTime_actual
                    realTime_anterior = realTime_actual

                    contador = contador + 1

                    #mientras no lleguemos, siga avanzando
                    while rho > 0.3 and connect == True and Quadricopter_base != 0:
                        pub_irrigation_flag.publish('B30L')
                        simTime_actual = simTime
                        realTime_actual = realTime

                        #delta_simTime = simTime_actual - simTime_anterior
                        #delta_realTime = realTime_actual - realTime_anterior

                        #Actualizamos posiciones
                        pos_x, pos_y, pos_z = get_position()
                        ang_x, ang_y, ang_z = get_orientation()

                        deltaX = endPos[0] - pos_x #distancias
                        deltaY = endPos[1] - pos_y

                        rho = np.sqrt(deltaX**2 + deltaY**2)

                        #print('Pose: ' + str(round(pos_x,3)) + ' ' + str(round(pos_y,3)) + ' ' + str(round(pos_z,3)) + ' | RHO: ' + str(round(rho,3)) +' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) +'| VEL: '+ str(round(paso_x,3)) +', ' + str(round(paso_y,3))+'| Deltas: '+ str(round(deltaX,3)) +', ' + str(round(deltaY,3)))
                        print(str(velocity) + ' | real: ' + str(round(delta_realTime,4)) +' simTime: ' + str(round(delta_simTime,4)) + ' Time: ' + str(round(tiempito,3)) + ' | RHO: ' + str(round(rho,3)) +' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) )#+  ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(pos_z,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))
                        sys.stdout.write("\033[K") # Clear to the end of line
                        sys.stdout.write("\033[F") # Cursor up one line

                        avance.data = [pos_x+v_x, pos_y+v_y, endPos[2]]
                        avance_eu.data = [ang_x, ang_y, ang_z]

                        pub_pose.publish(avance)
                        pub_euler.publish(avance_eu)

                    delta_simTime = simTime_actual - simTime_anterior
                    delta_realTime = realTime_actual - realTime_anterior
                    print('real: ' + str(round(delta_realTime,4)) +' simTime: ' + str(round(delta_simTime,4)) + ' Time: ' + str(round(tiempito,3)) + ' | RHO: ' + str(round(rho,3)) +' EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) )#+  ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(pos_z,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))
                    #sys.stdout.write("\033[K") # Clear to the end of line
                    #sys.stdout.write("\033[F") # Cursor up one line


        print('----------------------------Termine la ruta--------------------------')
        #sys.stdout.write("\033[K") # Clear to the end of line
        #sys.stdout.write("\033[F") # Cursor up one line
        rate.sleep()

#main_control()

if __name__ == '__main__':
	try:
		main_control()
	except rospy.ROSInterruptException:
		print('Nodo detenido')