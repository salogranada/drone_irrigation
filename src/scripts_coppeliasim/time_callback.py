#!/usr/bin/env python
import numpy as np
import sys
import time
import rospy
from std_msgs.msg import Float32

global simTime_anterior, realTime_anterior, realTime_actual, simTime_actual

simTime_anterior = 0
realTime_anterior = 0
simTime_actual = 0
realTime_actual = 0

#Obtenemos el tiempo de simulacion
def simTime_callback(msg):
    global simTime_actual
    simTime_actual = msg.data

#Obtenemos tiempo real
def realTime_callback(msg):
    global realTime_actual
    realTime_actual = msg.data

def main():
    print('Starting time node...')
    print(' ')
    rospy.init_node('time', anonymous=True) #Inicio nodo
    rate = rospy.Rate(10) #10hz

    #Subscripcion a topicos
    rospy.Subscriber("/simulationTime", Float32, simTime_callback, tcp_nodelay=True)
    rospy.Subscriber("/realTime", Float32, realTime_callback, tcp_nodelay=True)

    #Publicacion de topicos
    pub_realTime = rospy.Publisher('drones/delta_realTime', Float32, queue_size=10)
    pub_simTime = rospy.Publisher('drones/delta_simTime', Float32, queue_size=10)

    while not rospy.is_shutdown():
        global simTime_anterior, realTime_anterior, realTime_actual, simTime_actual

        #Calculamos tiempo en simulacion y tiempo real
        delta_simTime = simTime_actual - simTime_anterior
        delta_realTime = realTime_actual - realTime_anterior

        simTime_anterior = simTime_actual
        realTime_anterior = realTime_actual

    rate.sleep()
