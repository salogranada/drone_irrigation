#!/usr/bin/env python3
import os
import pandas as pd

#Joined dataframe
total_features_df = pd.DataFrame()

for filename in os.listdir("../data_base/paths_energy/"):
    #Change 'startswith' or 'endswith' depending on what data you want to join
    if filename.startswith("feat_"): 
        print(filename)
        energy_df = pd.read_csv('../data_base/paths_energy/' + filename, sep=",")
        total_features_df = total_features_df.append(energy_df, ignore_index=True)
        #print(total_features_df)

total_features_df.to_csv('../data_base/paths_energy/joined_features_11y12.csv') #Change name
