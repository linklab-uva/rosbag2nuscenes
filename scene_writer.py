#!/usr/bin/python3

import sys
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from secrets import token_hex
import json

allowable_tracks = {'LVMS', 'IMS', 'LOR'}

def write_scene(rosbag_file, track_name):
    if track_name not in allowable_tracks:
        print("Track name not recognized, must be LVMS, IMS, or LOR")
        exit(1)
    with Reader(rosbag_file) as reader:
        # Create scene.json
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/novatel_top/dyntf_odom':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                print(msg.header.frame_id)

if __name__=="__main__":
    if len(sys.argv) != 3:
        print("Expecting: [rosbag path] [track name]")
        print("Got: ", end='')
        for arg in sys.argv:
            print(arg, end='')
        exit(1)
    write_scene(sys.argv[1], sys.argv[2])