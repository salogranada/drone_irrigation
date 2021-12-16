#!/usr/bin/env python3
import numpy as np
import pandas as pd

#Reads the data file gotten from simulation. Calculates energy consumption; path time, distance and avg__vel.
#Author: Salomon Granada Ulloque
#E-mail: s.granada@uniandes.edu.co

#****************************************
#FOR ANY NEW DATA YOU HAVE TO SPECIFY 3 FILES! flight_df (flight data), path_df (random path data), energy_df (where you save the output)
#****************************************

print('Starting energy calculation... wait till its done.')

#Opens data frame for the flight data
flight_df = pd.read_csv('../data_base/flight_data/prueba_paths.txt', sep="|")
flight_df.columns = flight_df.columns.str.strip()

#Removes simulation time data that is < 0, indicates mal functioning
flight_df = flight_df.loc[flight_df['simTime'] > 0]

#Opens data frame for the  random_paths file.
path_df = pd.read_csv('../data_base/random_paths/prueba_paths_header.txt', sep="|")

#Opens data frame for error file.
error_df = pd.read_csv('../data_base/reports/prueba_paths.txt', sep="|")

#Creates new data frame for saving energies
energy_df = pd.DataFrame()  

#Creates new data frame for faving total values.
totals_df = pd.DataFrame()

dist_list = []
distX_list = []
distY_list = []

total_times_list = []

m1_sum_torque, m1_avg_torque = 0,0
m2_sum_torque, m2_avg_torque = 0,0
m3_sum_torque, m3_avg_torque = 0,0
m4_sum_torque, m4_avg_torque = 0,0

#Go through each of the random generated paths (path_df)
for idex_path, row_path_specs in path_df.iterrows():
    
    total_time_path = 0
    coordX_old, coordY_old = 0, 0
    sum_dist = 0
    intPoints_list =[]

    current_path = int(row_path_specs.loc['path_num'])

    #Retrieves lists of times for each path
    time_list = list(map(int,row_path_specs.loc['times'].split(','))) #convert time strings to int

    #Retrieves lists of points for each path
    points_list = list(row_path_specs.loc['points'].split(';')) 
    for i in points_list:
        intPoints = list(map(int,i.split(','))) #convert time strings to int
        intPoints_list.append(intPoints)

    #Goes through each of the points in each of the paths (path_df)
    for current_point in range(len(intPoints_list)-1,-1,-1):

        #Restarts sums for each path
        m1_sum_rpm,m2_sum_rpm, m3_sum_rpm,m4_sum_rpm = 0,0,0,0
        m1_sum_torque,m2_sum_torque, m3_sum_torque,m4_sum_torque = 0,0,0,0
        energy = 0

        #------------------------------
        #Total TIME in path calculation
        #------------------------------
        #Adds all the times required to end current path
        current_time = time_list[len(time_list)-1-current_point]
        total_time_path = total_time_path + current_time
        #--------------------------
        #Total distance in path calculation
        #--------------------------
        #Current coord in X and Y
        coordX_new = intPoints_list[current_point][0]
        coordY_new = intPoints_list[current_point][1]
        #Computes distance from one point to another.
        current_point_dist = np.sqrt((coordX_new-coordX_old)**2 + (coordY_new-coordY_old)**2)
        current_point_Xdist = coordX_new-coordX_old
        current_point_Ydist = coordY_new-coordY_old

        sum_dist = sum_dist + current_point_dist
        sum_distX = sum_distX + current_point_Xdist
        sum_distY = sum_distY + current_point_Ydist
        #Update current X and Y coord
        coordX_old = coordX_new
        coordY_old = coordY_new


        #-----------------------------------------------------------------
        #Create data frame with flight data from the current point in path
        #-----------------------------------------------------------------
        current_pathPoint_info = flight_df.loc[(flight_df['route_num'] == current_path) & (flight_df['missing_points'] == current_point)]

        #Some points of paths doesnt have flight data, dont try to do operations if no data.
        if not current_pathPoint_info.empty:
            
            #Help us check if current point is in error file. If it is, dont calculate energy.
            error_point = error_df.loc[( error_df['route_num'] == current_path )&(error_df['point'] == current_point )]
            if error_point.empty:

                #Iters through all data in one of the points in flight_data.txt file
                for index, row_point_specs in current_pathPoint_info.iterrows():

                    #Gets how much time the drone lasted in that point
                    sim_drone_time = current_pathPoint_info['simTime'].iloc[-1]

                    m1_rpm, m2_rpm, m3_rpm, m4_rpm = row_point_specs['rpm_list'].split(',')
                    m1_torque, m2_torque, m3_torque, m4_torque = row_point_specs['torque_array'].split(',')

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

                energy = np.abs(np.dot(np.array(total_rpm), np.array(total_torque)))
                teorical_vel = current_point_dist/current_time

                #print(total_rpm, total_torque, energy)

                #Only save those that really used energy.
                if energy != 0:
                    new_df = pd.DataFrame([[current_path, current_point,current_time,sim_drone_time,current_point_dist,current_point_Xdist, current_point_Ydist, teorical_vel,energy]], columns=['path_num', 'missing_points','teo_point_time', 'sim_drone_time', 'teo_point_dist','teo_Xdist','teo_Ydist', 'teo_point_vel','Energy'])
                    energy_df = energy_df.append(new_df, ignore_index=True)
                    #print(energy_df)

    #Saves total dist in the path, total and for each axis.
    dist_list.append(sum_dist)
    distX_list.append(sum_distX)
    distY_list.append(sum_distY)
    total_times_list.append(total_time_path)
    avg_path_vel = np.array(dist_list)/np.array(total_times_list)

    data_df = pd.DataFrame([[current_path]], columns=['path_num'])
    totals_df = totals_df.append(data_df, ignore_index=True)

#Add totals lists to the totals data frame.
totals_df['teo_path_dist'] = dist_list
totals_df['teo_path_Xdist'] = distX_list
totals_df['teo_path_Ydist'] = distY_list
totals_df['teo_path_time'] = total_times_list
totals_df['avg_path_vel'] = avg_path_vel

print(energy_df)
print(totals_df)

#Save to CSV file
energy_df.to_csv('../data_base/paths_energy/prueba_paths.csv',index=False)
totals_df.to_csv('../data_base/paths_totals/prueba_paths_totals.csv',index=False)
