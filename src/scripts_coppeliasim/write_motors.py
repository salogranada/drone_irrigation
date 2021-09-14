#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import sys
import numpy as np

# PERMITE ESCRIBIRLE UNA VELOCIDAD A LOS MOTORES DEL DRON
# Funciona con ros_test_manualVelLUA.ttt
# TESIS

global pos_x, pos_y, theta

pos_x = 0
pos_y = 0
theta = 0

def orientation_callback(msg): #Me regresa el angulo en el que se encuentra orientado en el marco robot
	global theta
	theta = msg.data


def position_callback(msg): #Me regresa la posicion en el marco inercial del robot
	global pos_x, pos_y
	pos_x = msg.linear.x 
	pos_y = msg.linear.y 

def simTime_callback(msg):
	global t, t_total, rho_total, rho
	t = msg.data

#Funcion principal de movimiento
def main_write():
    global pos_x, pos_y, theta
    
    rospy.init_node('moveloo', anonymous=True) #Inicio nodo
    endPos = [-2,-2, -np.pi*3/4] #Posicion final por defecto

    if len(sys.argv) > 2: #Utilizando la posicion final entrada por parametro
        endPos[0] = float(sys.argv[1])
        endPos[1] = float(sys.argv[2])
        endPos[2] = float(sys.argv[3])

    pub = rospy.Publisher('drone_wheelsVel', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) #10hz
    rospy.Subscriber("turtlebot_position", Twist, position_callback)
    rospy.Subscriber("simulationTime", Float32, simTime_callback)
    rospy.Subscriber("turtlebot_orientation", Float32, orientation_callback)

    ruedas = Float32MultiArray()
    ruedas.data = [0.0, 0.0, 0.0, 0.0]

    while not rospy.is_shutdown():
        ruedas.data = [5.3, 5.3, 5.3, 5.3]
        pub.publish(ruedas)
        print(ruedas)
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line

        rate.sleep()

if __name__ == '__main__':
	main_write()