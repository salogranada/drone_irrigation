#!/usr/bin/env python3
import numpy as np

#Reads the data file gotten from simulation. Calculates energy consumption of the drone.
#Author: Salomon Granada Ulloque
#E-mail: s.granada@uniandes.edu.co

#Opens data file.
flight_data = '../main_scripts/flight_data.txt'
f = open(flight_data, "r")
line = f.readline() #route_num|missing_points|simTime|tankMass|force_array|torque_array|rpm_list|droneVel
line = f.readline()
#print(line)

current_route = 1
avg_rpm = []
m1_sum_rpm, m1_avg_rpm = 0,0
m2_sum_rpm, m2_avg_rpm = 0,0
m3_sum_rpm, m3_avg_rpm = 0,0
m4_sum_rpm, m4_avg_rpm = 0,0

m1_sum_torque, m1_avg_torque = 0,0
m2_sum_torque, m2_avg_torque = 0,0
m3_sum_torque, m3_avg_torque = 0,0
m4_sum_torque, m4_avg_torque = 0,0

energy_dict = {}

while True:
    
    data = line.split('|')
    route_num,missing_points,simTime,tankMass,force_array,torque_array,rpm_list,droneVel = data

    route_num =  int(float(route_num))
    missing_points = int(float(missing_points))
    simTime = int(float(simTime))
    tankMass = int(float(tankMass))

    m1_force, m2_force, m3_force, m4_force = force_array.split(',')
    m1_torque, m2_torque, m3_torque, m4_torque = torque_array.split(',')
    m1_rpm, m2_rpm, m3_rpm, m4_rpm = rpm_list.split(',')

    m1_rpm = m1_rpm.split('[')
    m1_rpm = m1_rpm[1].replace('[','')

    m4_rpm = m4_rpm.split(']')
    m4_rpm = m4_rpm[0].replace(']','')

    m1_torque = m1_torque.split('[')
    m1_torque = m1_torque[1].replace('[','')

    m4_torque = m4_torque.split(']')
    m4_torque = m4_torque[0].replace(']','')

    if route_num != current_route:

        avg_rpm = [m1_sum_rpm, m2_sum_rpm, m3_sum_rpm, m4_sum_rpm]
        avg_torque = [m1_sum_torque, m2_sum_torque, m3_sum_torque, m4_sum_torque]

        energy = np.abs(np.dot(np.array(avg_rpm), np.array(avg_torque)))

        energy_dict[current_route] = energy
        #print('energy: ', energy_dict)

        m1_sum_rpm, m1_avg_rpm = 0,0
        m2_sum_rpm, m2_avg_rpm = 0,0
        m3_sum_rpm, m3_avg_rpm = 0,0
        m4_sum_rpm, m4_avg_rpm = 0,0

        m1_sum_torque, m1_avg_torque = 0,0
        m2_sum_torque, m2_avg_torque = 0,0
        m3_sum_torque, m3_avg_torque = 0,0
        m4_sum_torque, m4_avg_torque = 0,0

    m1_sum_rpm = m1_sum_rpm + int(float(m1_rpm))
    m2_sum_rpm = m2_sum_rpm + int(float(m2_rpm))
    m3_sum_rpm = m3_sum_rpm + int(float(m3_rpm))
    m4_sum_rpm = m4_sum_rpm + int(float(m4_rpm))

    m1_sum_torque = m1_sum_torque + int(float(m1_torque))
    m2_sum_torque = m2_sum_torque + int(float(m2_torque))
    m3_sum_torque = m3_sum_torque + int(float(m3_torque))
    m4_sum_torque = m4_sum_torque + int(float(m4_torque))
    
    current_route = route_num
    line = f.readline()
    if line == '':
        avg_rpm = [m1_sum_rpm, m2_sum_rpm, m3_sum_rpm, m4_sum_rpm]
        avg_torque = [m1_sum_torque, m2_sum_torque, m3_sum_torque, m4_sum_torque]

        energy = np.abs(np.dot(np.array(avg_rpm), np.array(avg_torque)))
        energy_dict[current_route] = energy

        print('energy: ', energy_dict)
        break
