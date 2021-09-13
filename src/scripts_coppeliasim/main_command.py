#!/usr/bin/env python
import rospy
import sys
import numpy as np
from std_msgs.msg import Empty, Float32MultiArray, String, Float32
from geometry_msgs.msg import Twist, Vector3

#Crear otro nodo para publicar la ruta
# - Por un lado publicar pose x,y,z y por otro euler de orientacion
#Publicar el numero de puntos faltantes
#Publicar el estado del breaker
#Publicar el modo en que se esta "no turn", turn+move, .....

def main_write():

    #Starting main_command node
    rospy.init_node('main_command', anonymous=True)

    #Starting publishers
    pub_breaker_state = rospy.Publisher('/autopilot_breaker_state', Empty, queue_size=10)
    pub_newpos = rospy.Publisher('/newpos', Float32MultiArray, queue_size=10)
    pub_neworientation = rospy.Publisher('/neworientation', Float32MultiArray, queue_size=10)
    pub_no_of_points = rospy.Publisher('/no_of_points', Float32, queue_size=10)
    pub_autopilot_mode = rospy.Publisher('/autopilot_mode', String, queue_size=10)
    rate = rospy.Rate(10) #10hz

    ruta = np.array([[-4,-4], [-4, 4], [-2, 4], [-2, -4], [0,-4], [0,4], [2,4], [2, 0], [3, 0], [3, 4], [4,4], [-4,-4]])


    while not rospy.is_shutdown():

        pub_breaker_state.publish(False)
        pub_no_of_points.publish(len(ruta))
        for point in ruta:
            pub_newpos.publish(point)
            pub_neworientation.publish([0,0,0])

        pub_autopilot_mode.publish('turn&move')

        print('publishing...')
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line
        
        rate.sleep()
        


if __name__ == '__main__':
	main_write()