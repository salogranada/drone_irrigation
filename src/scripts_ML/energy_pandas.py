#!/usr/bin/env python3
from os import path
import numpy as np
import pandas as pd

#Reads the data file gotten from simulation. Calculates energy consumption of the drone.
#Author: Salomon Granada Ulloque
#E-mail: s.granada@uniandes.edu.co

m1_sum_torque, m1_avg_torque = 0,0
m2_sum_torque, m2_avg_torque = 0,0
m3_sum_torque, m3_avg_torque = 0,0
m4_sum_torque, m4_avg_torque = 0,0

#Creates data frame for the flight data and the original random_paths file.
flight_df = pd.read_csv('../data_base/flight_data/flight_data_viejito.txt', sep="|")
flight_df.columns = flight_df.columns.str.strip()

#Remove simulation time data that is < 0, indicates mal functioning
flight_df = flight_df.loc[flight_df['simTime'] > 0]

path_df = pd.read_csv('../data_base/random_paths/path_v3_header_20faltantes.txt', sep="|")

#New data fram with energies
energy_df = path_df
energy_df['Energy'] = 0

#Go through each of the random generated paths
for i, row_path_specs in path_df.iterrows():

    #Restarts sums for each path
    m1_sum_rpm,m2_sum_rpm, m3_sum_rpm,m4_sum_rpm = 0,0,0,0
    m1_sum_torque,m2_sum_torque, m3_sum_torque,m4_sum_torque = 0,0,0,0

    #Retrieves lists of times for each path
    time_list = list(map(int,row_path_specs.loc['times'].split(','))) #convert time strings to int

    #Selects current path data from the whole list of paths
    current_path = flight_df.loc[flight_df['route_num'] == int(row_path_specs.loc['path_num'])]

    #Iters through all data in one of the paths in flight_data.txt file
    for index, row in current_path.iterrows():

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

        print(row['route_num'])
        #print("m1_rpm: ",m1_rpm)
        #print("m1_torque: ",m1_torque)

        m1_sum_rpm = m1_sum_rpm + int(float(m1_rpm))
        m2_sum_rpm = m2_sum_rpm + int(float(m2_rpm))
        m3_sum_rpm = m3_sum_rpm + int(float(m3_rpm))
        m4_sum_rpm = m4_sum_rpm + int(float(m4_rpm))

        m1_sum_torque = m1_sum_torque + int(float(m1_torque))
        m2_sum_torque = m2_sum_torque + int(float(m2_torque))
        m3_sum_torque = m3_sum_torque + int(float(m3_torque))
        m4_sum_torque = m4_sum_torque + int(float(m4_torque))

        total_rpm = [m1_sum_rpm, m2_sum_rpm, m3_sum_rpm, m4_sum_rpm]
        total_torque = [m1_sum_torque, m2_sum_torque, m3_sum_torque, m4_sum_torque]

        energy = np.abs(np.dot(np.array(total_rpm), np.array(total_torque)))

        energy_df.loc[energy_df.index[i], 'Energy'] = energy

energy_df.to_csv('../data_base/paths_energy/paths_energy_20faltantes.csv',index=False)
