#!/usr/bin/env python3
from os import path
import time
import numpy as np
import pandas as pd

#Reads the data file gotten from simulation. Calculates energy consumption; path time, distance and avg__vel.
#Author: Salomon Granada Ulloque
#E-mail: s.granada@uniandes.edu.co

#****************************************
#FOR ANY NEW DATA YOU HAVE TO SPECIFY 3 FILES! flight_df (flight data), path_df (random path data), energy_df (where you save the output)
#****************************************

m1_sum_torque, m1_avg_torque = 0,0
m2_sum_torque, m2_avg_torque = 0,0
m3_sum_torque, m3_avg_torque = 0,0
m4_sum_torque, m4_avg_torque = 0,0

#Creates data frame for the flight data
flight_df = pd.read_csv('../data_base/flight_data/flight_data_2000_parte2.txt', sep="|")
flight_df.columns = flight_df.columns.str.strip()

#Remove simulation time data that is < 0, indicates mal functioning
flight_df = flight_df.loc[flight_df['simTime'] > 0]

#Creates data frame for the  random_paths file.
path_df = pd.read_csv('../data_base/random_paths/path_v4_header_p2.txt', sep="|")

#New data frame with energies
energy_df = path_df
energy_df['Energy'] = 0
dist_list = []
total_times_list = []


#Go through each of the random generated paths (path_df)
for iter, row_path_specs in path_df.iterrows():

    #Restarts sums for each path
    m1_sum_rpm,m2_sum_rpm, m3_sum_rpm,m4_sum_rpm = 0,0,0,0
    m1_sum_torque,m2_sum_torque, m3_sum_torque,m4_sum_torque = 0,0,0,0
    total_time_path = 0
    coordX_old, coordY_old = 0, 0
    sum_dist = 0
    intPoints_list =[]


    #Retrieves lists of points for each path
    points_list = list(row_path_specs.loc['points'].split(';')) 

    for i in points_list:
        intPoints = list(map(int,i.split(','))) #convert time strings to int
        intPoints_list.append(intPoints)
    
    for i in intPoints_list:
        #Current coord in X and Y
        coordX_new = i[0]
        coordY_new = i[1]

        current_point_dist = np.sqrt((coordX_new-coordX_old)**2 + (coordY_new-coordY_old)**2)
        sum_dist = sum_dist + current_point_dist

        #Update current X and Y coord
        coordX_old = coordX_new
        coordY_old = coordY_new
    
    #Saves total dist in the path
    dist_list.append(sum_dist)


    #Retrieves lists of times for each path
    time_list = list(map(int,row_path_specs.loc['times'].split(','))) #convert time strings to int

    #Adds all the times required to end current path
    for i in time_list:
        total_time_path = total_time_path + i
    
    total_times_list.append(total_time_path)

    #Gets current path flight_data (DATA FRAME)
    current_path_info = flight_df.loc[flight_df['route_num'] == int(row_path_specs.loc['path_num'])]

    #print(current_path_info['simTime'].cumsum())

    drone_real_time = current_path_info['simTime'].iloc[-1]

    print('final time is: ', drone_real_time)

    #Iters through all data in one of the paths in flight_data.txt file
    for index, row in current_path_info.iterrows():

        m1_rpm, m2_rpm, m3_rpm, m4_rpm = row['rpm_list'].split(',')
        m1_torque, m2_torque, m3_torque, m4_torque = row['torque_array'].split(',')

        #Clean obtained string
        m1_rpm = m1_rpm.split('[')
        m1_rpm = m1_rpm[1].replace('[','')
        m4_rpm = m4_rpm.split(']')
        m4_rpm = m4_rpm[0].replace(']','')

        m1_torque = m1_torque.split('[')
        m1_torque = m1_torque[1].replace('[','')
        m4_torque = m4_torque.split(']')
        m4_torque = m4_torque[0].replace(']','')

        m1_sum_rpm = m1_sum_rpm + float(float(m1_rpm))
        m2_sum_rpm = m2_sum_rpm + float(float(m2_rpm))
        m3_sum_rpm = m3_sum_rpm + float(float(m3_rpm))
        m4_sum_rpm = m4_sum_rpm + float(float(m4_rpm))

        m1_sum_torque = m1_sum_torque + abs(float(float(m1_torque)))
        m2_sum_torque = m2_sum_torque + abs(float(float(m2_torque)))
        m3_sum_torque = m3_sum_torque + abs(float(float(m3_torque)))
        m4_sum_torque = m4_sum_torque + abs(float(float(m4_torque)))

        total_rpm = [m1_sum_rpm, m2_sum_rpm, m3_sum_rpm, m4_sum_rpm]
        total_torque = [m1_sum_torque, m2_sum_torque, m3_sum_torque, m4_sum_torque]

        #print(total_rpm, '  torque: ',total_torque)

        energy = np.abs(np.dot(np.array(total_rpm), np.array(total_torque)))
        #print('ENERGY: ',energy)
        energy_df.loc[energy_df.index[iter], 'Energy'] = energy
        
avg_path_vel = np.array(dist_list)/np.array(total_times_list)

energy_df['total_dist'] = dist_list
energy_df['total_time'] = total_times_list
energy_df['avg_path_vel'] = avg_path_vel

print(energy_df)

#energy_df.to_csv('../data_base/paths_energy/paths_energy_2000_parte2_salo.csv',index=False)
