#!/usr/bin/env python3

import rospy 

from std_msgs.msg import Int32,String

def numero(cont):
    return cont

def key():
    return 'B30L'

def auxiliar():
    pub1 = rospy.Publisher('/simulationTime',Int32,queue_size=1)
    pub2 = rospy.Publisher('/irrigation_flag',String,queue_size=1)
    rospy.init_node('auxiliarNode')
    rate = rospy.Rate(1)
    print("Estoy activo")
    cont = 0
    while not rospy.is_shutdown():
        valor = numero(cont)
        flag = key()
        pub2.publish(flag)
        pub1.publish(valor)
        cont += 1
        rate.sleep()
        
if __name__ == '__main__':
    auxiliar()