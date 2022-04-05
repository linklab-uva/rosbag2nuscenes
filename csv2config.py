import sys
import os
import numpy as np
import yaml
import csv


def read_data():
    folder = "/home/john/Downloads/test.csv"
    csvfile = open(folder)
    reader = csv.reader(csvfile, delimiter=',', quotechar='"')
    data = []
    for row in reader:
        data.append(row)
    data.pop(0)
    global datetime, bag_name, team, description, data_category, lidar_names, lidar_segmented, gps_name, tf_name, radar_bool, radar_name, camera_bool, camera_names
    for strarr in data:
        datetime = strarr[0]
        bag_name = strarr[1]
        team = strarr[2]
        description = strarr[3]
        data_category = strarr[4]
        lidar_segmented = strarr[5]
        lidar_names = strarr[6]
        gps_name = strarr[7]
        tf_name = strarr[8]
        radar_bool = strarr[9]
        radar_name = strarr[10]
        camera_bool = strarr[11]
        camera_names = strarr[12]


    csvfile.close()

def write_config():
    print("hello")
    

if __name__ == "__main__":

    read_data()
    write_config()
