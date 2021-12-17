#!/usr/bin/env python3
import os
import pandas as pd

#Joined dataframe
total_energy_df = pd.DataFrame()

for filename in os.listdir("../data_base/paths_energy/"):
    if filename.endswith("salo.csv"): #Change depending on who captured data.
        print(filename)
        energy_df = pd.read_csv('../data_base/paths_energy/' + filename, sep=",")
        total_energy_df = total_energy_df.append(energy_df, ignore_index=True)
        #print(total_energy_df)

total_energy_df.to_csv('../data_base/paths_energy/joined_energy_salo.csv',index=False) #Change depending on who captured data.
