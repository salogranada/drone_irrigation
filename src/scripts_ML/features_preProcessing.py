#!/usr/bin/env python3
import numpy as np
import pandas as pd

#Reads the data files gotten from simulation. Computes ML features such as energy consumption, path times, distance and avg__vel. Both simulated and teorical.
#Author: Salomon Granada Ulloque
#E-mail: s.granada@uniandes.edu.co

#*********************************************
#FOR ANY NEW DATA YOU HAVE TO SPECIFY 5 FILES! 
#*********************************************
#INPUT
flight_data_filename = '../data_base/flight_data/fd_pathY.txt'
path_data_filename = '../data_base/random_paths/path_ye.txt'
errorLog_filename = '../data_base/reports/errorlog_pathY.txt'
#OUTPUT
energy_df_filename = '../data_base/paths_energy/features_pathY.csv' #energy_df (where you save the energy and features output)
totals_df_filename = '../data_base/paths_totals/totals_pathY.csv' #totals_df (where you save total (cummulative) output)
#*********************************************

print('Starting features calculation... wait till its done.')

#Opens data frame for the flight data
flight_df = pd.read_csv(flight_data_filename, sep="|")
flight_df.columns = flight_df.columns.str.strip()
#Removes simulation time data that is < 0, indicates mal functioning
#flight_df = flight_df.loc[flight_df['simTime'] > 0]

#Opens data frame for the  random_paths file.
path_df = pd.read_csv(path_data_filename, sep="|")

#Opens data frame for error file.
error_df = pd.read_csv(errorLog_filename, sep="|")
error_df.columns = error_df.columns.str.strip()
error_df = error_df.loc[error_df['log'] != 'STATIC tankmass ']

#Creates new data frame for saving energies
energy_df = pd.DataFrame()  

#Creates new data frame for faving total values.
totals_df = pd.DataFrame()

dist_list = []
distX_list = []
distY_list = []
energy_list = []

total_times_list = []

m1_sum_torque, m1_avg_torque = 0,0
m2_sum_torque, m2_avg_torque = 0,0
m3_sum_torque, m3_avg_torque = 0,0
m4_sum_torque, m4_avg_torque = 0,0

#Go through each of the random generated paths (path_df)
for idex_path, row_path_specs in path_df.iterrows():
    
    total_time_path = 0
    coordX_old, coordY_old = 0, 0
    sum_dist, sum_distX, sum_distY = 0,0,0
    intPoints_list =[]
    total_energy = 0

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
        m1_sum_force,m2_sum_force, m3_sum_force,m4_sum_force = 0,0,0,0
        m1_sum_rpm,m2_sum_rpm, m3_sum_rpm,m4_sum_rpm = 0,0,0,0
        m1_sum_torque,m2_sum_torque, m3_sum_torque,m4_sum_torque = 0,0,0,0
        rpm_list = [0,0,0,0]
        avg_rpm_list = [0,0,0,0]
        energy = 0
        energy_new = 0
        avg_count = 0
        sim_point_dist = 0
        old_pos_x, old_pos_y = 0,0

        #------------------------------
        #Total TEORICAL TIME in path calculation
        #------------------------------
        #Adds all the times required to end current path
        current_time = time_list[len(time_list)-1-current_point]
        total_time_path = total_time_path + current_time
        #--------------------------
        #Total TEORICAL distance in path calculation
        #--------------------------
        #Current coord in X and Y
        coordX_new = intPoints_list[current_point][0]
        coordY_new = intPoints_list[current_point][1]
        #Computes distance from one point to another.
        current_point_dist = np.sqrt((coordX_new-coordX_old)**2 + (coordY_new-coordY_old)**2)
        current_point_Xdist = np.sqrt((coordX_new-coordX_old)**2)
        current_point_Ydist = np.sqrt((coordY_new-coordY_old)**2)

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
            #if True:
                #--------------------------
                #Computes total SIMULATED xDisplacement and yDisplacement in one point.
                #--------------------------
                current_pathPoint_info['xDisplacement'].cumsum
                current_pathPoint_info['yDisplacement'].cumsum
                sim_Xdist = current_pathPoint_info['xDisplacement'].iloc[-1]
                sim_Ydist = current_pathPoint_info['yDisplacement'].iloc[-1]

                #--------------------------
                #Computes distance error between drone and target (particle)
                #--------------------------
                current_pathPoint_info['error_target_dist'].mean
                avg_error_target_dist = current_pathPoint_info['error_target_dist'].iloc[-1]
                #Iters through all data in one of the points in flight_data.txt file
                for index, row_point_specs in current_pathPoint_info.iterrows():
                    
                    #--------------------------
                    #Computes simulated traveled distance
                    #--------------------------
                    curr_pos_x = row_point_specs['pos_x']
                    curr_pos_y = row_point_specs['pos_y']
                    travel_dist = np.sqrt((curr_pos_x-old_pos_x)**2 + (curr_pos_y-old_pos_y)**2)
                    sim_point_dist = sim_point_dist + travel_dist
                    old_pos_x = curr_pos_x
                    old_pos_y = curr_pos_y

                    #RPM calculation for each motor. 
                    c = 0.6
                    e_d = 0.88 #diameter effectiveness
                    theta = 25 #blade twist angle (deg)
                    p = 1.225 #air density kg/m^3
                    R = (1.4974e-01)/2 #blade radio
                    k = 2*c/4*R #Motor-propeller Force Constant
                    C_T = (4/3)*k*theta*(1-(1-e_d)**3) - k*(np.sqrt(k*(k+1))- np.sqrt(k))*(1-(1-e_d)**2)

                    #RPM for each motor calculation. 
                    c = 0.6
                    e_d = 0.88 #diameter effectiveness
                    theta = 25 #blade twist angle (deg)
                    p = 1.225 #air density kg/m^3
                    R = (1.4974e-01)/2 #blade radio
                    k = 2*c/4*R #Motor-propeller Force Constant
                    C_T = (4/3)*k*theta*(1-(1-e_d)**3) - k*(np.sqrt(k*(k+1))- np.sqrt(k))*(1-(1-e_d)**2)

                    #Gets how much time the drone lasted in that point
                    sim_drone_time = current_pathPoint_info['simTime'].iloc[-1]

                    #Gets force, rpm and torque arrays.
                    m1_force, m2_force, m3_force, m4_force = row_point_specs['force_array'].split(',')
                    m1_rpm, m2_rpm, m3_rpm, m4_rpm = row_point_specs['rpm_list'].split(',')
                    m1_torque, m2_torque, m3_torque, m4_torque = row_point_specs['torque_array'].split(',')

                    avg_count = avg_count + 1

                    #Clean obtained strings
                    m1_force = m1_force.split('[')
                    m1_force = m1_force[1].replace('[','')
                    m4_force = m4_force.split(']')
                    m4_force = m4_force[0].replace(']','')

                    m1_rpm = m1_rpm.split('[')
                    m1_rpm = m1_rpm[1].replace('[','')
                    m4_rpm = m4_rpm.split(']')
                    m4_rpm = m4_rpm[0].replace(']','')

                    m1_torque = m1_torque.split('[')
                    m1_torque = m1_torque[1].replace('[','')
                    m4_torque = m4_torque.split(']')
                    m4_torque = m4_torque[0].replace(']','')

                    #Adds all exerted force in current point.
                    m1_sum_force = m1_sum_force + abs(float(float(m1_force)))
                    m2_sum_force = m2_sum_force + abs(float(float(m2_force)))
                    m3_sum_force = m3_sum_force + abs(float(float(m3_force)))
                    m4_sum_force = m4_sum_force + abs(float(float(m4_force)))

                    m1_avg_force = m1_sum_force/avg_count
                    m2_avg_force = m2_sum_force/avg_count
                    m3_avg_force = m3_sum_force/avg_count
                    m4_avg_force = m4_sum_force/avg_count

                    m1_sum_rpm = m1_sum_rpm + abs(float(float(m1_rpm)))
                    m2_sum_rpm = m2_sum_rpm + abs(float(float(m2_rpm)))
                    m3_sum_rpm = m3_sum_rpm + abs(float(float(m3_rpm)))
                    m4_sum_rpm = m4_sum_rpm + abs(float(float(m4_rpm)))

                    m1_sum_torque = m1_sum_torque + abs(float(float(m1_torque)))
                    m2_sum_torque = m2_sum_torque + abs(float(float(m2_torque)))
                    m3_sum_torque = m3_sum_torque + abs(float(float(m3_torque)))
                    m4_sum_torque = m4_sum_torque + abs(float(float(m4_torque)))

                #For each of the motors calculates its RPM depending on the exerted force.
                force = [m1_sum_force, m2_sum_force, m3_sum_force, m4_sum_force]
                avg_force = [m1_avg_force, m2_avg_force, m3_avg_force, m4_avg_force]

                for motor in range(len(force)):
                    rpm = np.sqrt(abs(force[motor])/((1/16)*p*np.pi*(R**4)*(e_d**4)*C_T))
                    avg_rpm = np.sqrt(abs(avg_force[motor])/((1/16)*p*np.pi*(R**4)*(e_d**4)*C_T))
                    rpm_list[motor] = rpm
                    avg_rpm_list[motor] = avg_rpm

                total_rpm = [m1_sum_rpm, m2_sum_rpm, m3_sum_rpm, m4_sum_rpm]
                total_torque = [m1_sum_torque, m2_sum_torque, m3_sum_torque, m4_sum_torque]

                energy = np.abs(np.dot(np.array(total_rpm), np.array(total_torque)))
                energy_new = np.abs(np.dot(np.array(rpm_list), np.array(total_torque)))
                avg_energy = np.abs(np.dot(np.array(avg_rpm_list), np.array(total_torque)))

                teorical_vel = current_point_dist/current_time

                total_energy = total_energy + energy_new

                #Only save those that really used energy.
                if energy != 0:
                    new_df = pd.DataFrame([[current_path, current_point,current_time,sim_drone_time,current_point_dist,sim_point_dist,current_point_Xdist, current_point_Ydist,sim_Xdist,sim_Ydist,avg_error_target_dist,teorical_vel,energy, energy_new, avg_energy]], columns=['path_num', 'missing_points','teo_point_time', 'sim_drone_time', 'teo_point_dist','sim_point_dist','teo_Xdist','teo_Ydist','sim_Xdist','sim_Ydist','avg_error_target_dist','teo_point_vel','Energy','energy_new','avg_energy'])
                    energy_df = energy_df.append(new_df, ignore_index=True)
                    #print(energy_df)

    #Saves total dist in the path, total and for each axis.
    dist_list.append(sum_dist)
    distX_list.append(sum_distX)
    distY_list.append(sum_distY)
    total_times_list.append(total_time_path)
    avg_path_vel = np.array(dist_list)/np.array(total_times_list)
    energy_list.append(total_energy)

    data_df = pd.DataFrame([[current_path]], columns=['path_num'])
    totals_df = totals_df.append(data_df, ignore_index=True)

#Add totals lists to the totals data frame.
totals_df['teo_path_dist'] = dist_list
totals_df['teo_path_Xdist'] = distX_list
totals_df['teo_path_Ydist'] = distY_list
totals_df['teo_path_time'] = total_times_list
totals_df['avg_path_vel'] = avg_path_vel
totals_df['energy'] = energy_list

#Remove data with energy = 0, indicates that the drone didnt make that path
totals_df = totals_df.loc[totals_df['energy'] > 0]

#Save to CSV file
energy_df.to_csv(energy_df_filename,index=False)
totals_df.to_csv(totals_df_filename,index=False)

print('Saved features to: ', energy_df_filename)
print('Saved totals to: ', totals_df_filename)