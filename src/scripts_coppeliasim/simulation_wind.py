#!/usr/bin/env python3
 
import rospy
import sim
from std_msgs.msg import String,Float32,Int32
import random
import numpy as np



class windCity():
    
    def __init__(self):
        parametros_bogota = {'Bogota_vel_wind':np.array([1.66667,1.19445]),'Bogota_den_air':np.array([0.87899,0.90383]),'Bogota_press':np.array([73713.14,72683.44])}
        parametros_ejeCafetero = {'ejeCafetero_vel_wind':np.array([1.4722,1.13889]),'ejeCafetero_den_air':np.array([0.985415,1.010218])}
        parametros_tumaco = {'Tumaco_vel_wind':np.array([3.805556,2.88889]),'Tumaco_den_air':np.array([1.164398,1.18791])}
        self.parametros = {**parametros_bogota,**parametros_ejeCafetero,**parametros_tumaco}
        #######################################################################################
        """Solo va a escoger entre Bogota y ejeCafetero mientras se mejora el controlador"""
        name = random.randint(0,1)
        #######################################################################################
        names = ['Bogota','ejeCafetero','Tumaco']
        self.mode_city = names[name]
        #Direccion del viento: (0) x, (1) y, (2) -x, (3) -y
        self.mode_direction = random.randint(0,3)
        #Velocidad del viento: (0) min, (1) max
        self.speedWind = self.parametros[self.mode_city+'_vel_wind'][random.randint(0,1)]
        #Densidad del viento: (0) min, (1) max
        self.air_den = self.parametros[self.mode_city+'_den_air'][random.randint(0,1)]



class wind(object):

    def __init__(self,mode):
        self.clientID = sim.simxStart('127.0.0.1',19998,True,True,2000,5)
        self.subMass = rospy.Subscriber('/currentMass',Float32,self.airForce)
        print("self.clientID=", self.clientID == 0)
        _,self.handleCube = sim.simxGetObjectHandle(self.clientID,"Tank",sim.simx_opmode_blocking)
        
        if mode == 1:
            city = windCity()
            self.wind_vel_value = city.speedWind #wind velocity (m/s)
            self.air_den = city.air_den   #air density (kg/m3) 
            self.mode_direction = city.mode_direction
            print(f'Velocidad del viento {self.wind_vel}[m/s]')
            print(f'Densidad del aire {self.air_den}[kg/m3]')
            print(f'Direccion del viento {self.mode_direction}')
        else:
            self.wind_vel_value = 1.13889  #wind velocity (m/s)
            self.air_den = 1.010218   #air density (kg/m3)
            self.mode_direction = 3
             
        #Auxiliar variables
        self.counterDrag = 0
        self.coefficientDrag = 1.05
        self.areaCube = 0.015 
        self.mass_drone = 0.120 #Mass of the drone (kg)
        
    
    
    def DragForce(self):
        if self.counterDrag == 0:
            linearVelocity = sim.simxGetObjectVelocity(self.clientID,self.handleCube,sim.simx_opmode_streaming)
            linearVelocity = linearVelocity[1]
            print(f'Este es el vector velocidad {linearVelocity}')
            linearVelocity = np.array(linearVelocity,dtype=Float32)
            sign = np.where(linearVelocity < 0,-1,1)
            self.counterDrag += 1
        else:
            linearVelocity = sim.simxGetObjectVelocity(self.clientID,self.handleCube,sim.simx_opmode_buffer)
            linearVelocity = linearVelocity[1]
            print(f'Este es el vector velocidad {linearVelocity}')
            linearVelocity = np.array(linearVelocity,dtype=Float32)
            sign = np.where(linearVelocity < 0,-1,1)
        const = 0.5*self.coefficientDrag*self.air_den*self.areaCube
        linearVelocity = (linearVelocity[:-1]**2)*sign[:-1]
        return const*linearVelocity


    def airForce(self,dataMass):
        wind_vel = np.array([0.0,0.0])
        if self.mode_direction == 0 or self.mode_direction == 2:
            wind_vel[0] = self.wind_vel_value
            flag = 1
            print(wind_vel)
        elif self.mode_direction == 1 or self.mode_direction == 3:
            wind_vel[1] = self.wind_vel_value
            flag = 1
            print(wind_vel)
        elif self.mode_direction == 2 or self.mode_direction == 3:
            wind_vel = -1*self.wind_vel
            flag = -1
            print(wind_vel)

        air_force = flag*0.5*self.air_den*(wind_vel**2)*self.areaCube
        drag_force = self.DragForce()

        force = air_force + drag_force 
        acel = force/((dataMass.data/1000) + self.mass_drone)
        
        print("la fuerza es igual a ",air_force)
        print("la fuerza debido al drag ",drag_force)
        print("la aceleracion calculada es igual a ",acel)
        sim.simxSetArrayParameter(self.clientID,sim.sim_arrayparam_gravity,{acel[0],acel[1],-9.81},sim.simx_opmode_oneshot)
        




if __name__ == '__main__':
    rospy.init_node('wind',anonymous=True)
    print("Seleccionar modo default o Aleatorio: (0) default, (1) Aleatorio")
    wind = wind(mode=0)
    rospy.spin()