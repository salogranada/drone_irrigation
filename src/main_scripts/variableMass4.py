#!/usr/bin/env python3
 
import rospy
from std_msgs.msg import String,Float32,Int32

import numpy as np
from scipy.integrate import odeint
import math


class Tank:
    def __init__(self,dimTank,Q,den):
        self.dimTank = dimTank
        self.Q = Q #(m3/seg) Flow rate.
        self.den = den #(g/m3) Water density.
        self.h0 = dimTank[2] #(m) initial water height.
        self.AHole = (Q)/(math.sqrt(2*9.8*self.h0)) #(m2)Area of the hole where the water exits.
        self.ATank = dimTank[0]*dimTank[1] #(m2) Tank transversal area. 

def htank(h,t,a,A,c=1):
    #a -> area de la apertura de salida.
    #A -> area del tanque.
    #h -> Altura inicial del tanque.
    #c -> constante de velocidad del fluido.
    #t -> Tiempo.
    g = 9.8 #m/s2
    dhdt = -(a*c*np.sqrt(2*g*h))/A
    return dhdt

def mTank(dhdt,ATank,den):
    #den -> Fluid density.
    #dhdt-> Change in the water height.
    dmdt = den*dhdt*ATank
    return dmdt

def CaltimeSimulation(tank):
    return (2*(tank.ATank/tank.AHole)*math.sqrt(tank.h0))/(math.sqrt(2*9.8))

def close_time_value(array,value):
    idx = (np.abs(array - value)).argmin()
    return idx


class variableMass(object):
    
    def __init__(self):
        self.simulationTime = 0
        self.simTimeIrrigation0 = 0 #when the irrigation begins
        self.simulationTime_sub = rospy.Subscriber('/simulationTime',Float32,self.callback_simulationTime)
        self.pub = rospy.Publisher('/currentMass', Float32,queue_size=50)
        self.irrigationFlag = 'initial'
        self.irrigationFlag_sub = rospy.Subscriber('/PE/Drone/tank_volume', String, self.callback_activation)
        self.matrixValues = np.zeros((2,3))
        #print(self.matrixValues, "Esta es la matriz cuando inicia")
        self.h = 1
        self.m = 0
        self.t = 0 #linespace
        self.cont = 0
        

    ###############################################################################################
    ####Callbacks####
    ###############################################################################################

    def callback_simulationTime(self,dataTime):
        self.simulationTime = int(dataTime.data)
        #print("El tiempo actual es",dataTime.data)
    
    def callback_activation(self,dataFlag):
        self.irrigationFlag = dataFlag.data
        #Assuming the flow rate of the Agras MG-1, and the box dimensions of Agras T30 (WxL)
        if self.irrigationFlag == 'B10L': #Begins irrigation (Default 10L).
            print("Entro a 10L")
            if self.cont == 0:
                self.simTimeIrrigation0 = self.simulationTime
                self.cont += 1

            dimTank = np.array([0.560,0.435,(0.010)/(0.560*0.435)],dtype=np.float16)
            Q = 2.867e-5 #(m3/seg) Flow rate. 0.02867L/seg    
            den = 1e6 #(g/m3) Water density.
            tank10L = Tank(dimTank,Q,den)
            tFinal = CaltimeSimulation(tank10L) #Time in seconds.
            t = np.linspace(0,int(tFinal+5),int(tFinal+5)) #linspace with 5 seconds of excess
            h = odeint(htank,tank10L.h0,t,args=(tank10L.AHole,tank10L.ATank))
            m = mTank(h,tank10L.ATank,den)
            length = m.shape
            info = np.zeros((length[0],3))
            info[:,0] = h.reshape(-1)
            info[:,1] = m.reshape(-1)
            info[:,2] = t.reshape(-1)
            
            self.h = h
            self.m = m
            self.t = t
            self.matrixValues = info

        elif self.irrigationFlag == 'B15L': #Begins irrigation (15L).
            print("Entro a 15L")
            if self.cont == 0:
                self.simTimeIrrigation0 = self.simulationTime
                self.cont += 1

            dimTank = np.array([0.560,0.435,(0.015)/(0.560*0.435)],dtype=np.float16)
            Q = 2.867e-5 #(m3/seg) Flow rate. 0.02867L/seg
            den = 1e6 #(g/m3) Water density.
            tank15L = Tank(dimTank,Q,den)
            
            tFinal = CaltimeSimulation(tank15L) #Time in seconds.
            t = np.linspace(0,int(tFinal+5),int(tFinal+5)) #linspace with 5 seconds of excess
            
            h = odeint(htank,tank15L.h0,t,args=(tank15L.AHole,tank15L.ATank))
            m = mTank(h,tank15L.ATank,den)
            
            length = m.shape
            info = np.zeros((length[0],3))
            info[:,0] = h.reshape(-1)
            info[:,1] = m.reshape(-1)
            info[:,2] = t.reshape(-1)
            
            self.h = h
            self.m = m
            self.t = t
            self.matrixValues = info
            
        
        #Assuming the flow rate of the Agras T30, and the box dimensions of Agras T30 (WxL)
        elif self.irrigationFlag == 'B20L': #Begins irrigation (20L).

            if self.cont == 0:
                self.simTimeIrrigation0 = self.simulationTime
                self.cont += 1

            dimTank = np.array([0.560,0.435,(0.020)/(0.560*0.435)],dtype=np.float16)
            Q = 1.2e-4 #(m3/seg) Flow rate. 0.12L/seg
            den = 1e6 #(g/m3) Water density.
            tank20L = Tank(dimTank,Q,den)
            
            tFinal = CaltimeSimulation(tank20L) #Time in seconds.
            t = np.linspace(0,int(tFinal+5),int(tFinal+5)) #linspace with 5 seconds of excess
            
            h = odeint(htank,tank20L.h0,t,args=(tank20L.AHole,tank20L.ATank))
            m = mTank(h,tank20L.ATank,den)
            
            length = m.shape
            info = np.zeros((length[0],3))
            info[:,0] = h.reshape(-1)
            info[:,1] = m.reshape(-1)
            info[:,2] = t.reshape(-1)
            
            self.h = h
            self.m = m
            self.t = t
            self.matrixValues = info
        
        elif self.irrigationFlag == 'B25L': #Begins irrigation (25L).
            
            if self.cont == 0:
                self.simTimeIrrigation0 = self.simulationTime
                self.cont += 1
            
            dimTank = np.array([0.560,0.435,(0.025)/(0.560*0.435)],dtype=np.float16)
            Q = 1.2e-4 #(m3/seg) Flow rate. 0.12L/seg
            den = 1e6 #(g/m3) Water density.
            tank25L = Tank(dimTank,Q,den)
            
            tFinal = CaltimeSimulation(tank25L) #Time in seconds.
            t = np.linspace(0,int(tFinal+5),int(tFinal+5)) #linspace with 5 seconds of excess
            
            h = odeint(htank,tank25L.h0,t,args=(tank25L.AHole,tank25L.ATank))
            m = mTank(h,tank25L.ATank,den)
            
            length = m.shape
            info = np.zeros((length[0],3))
            info[:,0] = h.reshape(-1)
            info[:,1] = m.reshape(-1)
            info[:,2] = t.reshape(-1)
            
            self.h = h
            self.m = m
            self.t = t
            self.matrixValues = info
        
        elif self.irrigationFlag == 'B30L': #Begins irrigation (30L).
            if self.cont == 0:
                self.simTimeIrrigation0 = self.simulationTime
                self.cont += 1
            #print("El tiempo inicial es ", self.simTimeIrrigation0)
            dimTank = np.array([0.560,0.435,(0.030)/(0.560*0.435)],dtype=np.float16)
            Q = 1.2e-4 #(m3/seg) Flow rate. 0.12L/seg
            den = 1e6 #(g/m3) Water density.
            tank30L = Tank(dimTank,Q,den)
            
            tFinal = CaltimeSimulation(tank30L) #Time in seconds.
            t = np.linspace(0,int(tFinal+5),int(tFinal+5)) #linspace with 5 seconds of excess
            h = odeint(htank,tank30L.h0,t,args=(tank30L.AHole,tank30L.ATank))
            m = mTank(h,tank30L.ATank,den)
            
            length = m.shape
            info = np.zeros((length[0],3))
            info[:,0] = h.reshape(-1)
            info[:,1] = m.reshape(-1)
            info[:,2] = t.reshape(-1)
            
            self.h = h
            self.m = m
            self.t = t
            self.matrixValues = info

            #Publicar
            actual_time = self.simulationTime - self.simTimeIrrigation0 #in seconds
            #print("El delta de tiempo es igual a ",actual_time)
            idx = close_time_value(self.matrixValues[:,2],actual_time)
            #print("El tamaño de mi matriz es", self.matrixValues.shape)
            #print("Mi idx es igual a ", idx)
            self.pub.publish(self.matrixValues[idx,1])
            #print("Esta es la masa", self.matrixValues[idx,1])
            #print('\n')


            

if __name__ == '__main__':
    rospy.init_node('variableMass', anonymous=True)
    mass = variableMass()
    rospy.spin()