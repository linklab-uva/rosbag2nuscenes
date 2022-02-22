#!/usr/bin/python3

import sys
import os
from debugpy import log_to
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from secrets import token_hex
import json
from datetime import datetime

allowable_tracks = {'LVMS', 'IMS'}

def write_scene(rosbag_file, track_name):
    if track_name not in allowable_tracks:
        print("Track name not recognized, must be LVMS or IMS")
        exit(1)
    with Reader(rosbag_file) as reader:
        # Create scene.json
        first_scene, last_scene = '', ''
        log_token = ''
        with open('scene.json', 'w') as outfile:
            data = dict()
            data['token'] = token_hex(16)
            log_token = token_hex(16)
            data['log_token'] = log_token
            connection = [x for x in reader.connections.values() if x.topic == '/novatel_top/dyntf_odom'][0]
            data['nbr_samples'] = connection.count
            first_scene, last_scene = token_hex(16), token_hex(16)
            data['first_sample_token'] = first_scene
            data['last_sample_token'] = last_scene
            data['name'] = 'scene-0001'  # TODO: increase global variable to create name
            data['description'] = track_name
            json_string = json.dumps(data)
            outfile.write(json_string)
        # Create map.json
        # Case where map file exists
        if os.path.exists('map.json'):
            with open('map.json', 'r') as outfile:
                data = json.load(outfile)
            data['log_tokens'].append(log_token)
            with open('map.json', 'w') as outfile:
                json.dump(data,outfile)
        else:
        # Case where map.json does not exist
            with open('map.json', 'w') as outfile:
                data = dict()
                data['token'] = token_hex(16)
                data['log_tokens'] = []
                data['log_tokens'].append(log_token)
                data['category'] = track_name
                data['filename'] = "path/to/origin/data" # TODO: choose an origin to use
                json_string = json.dumps(data)
                outfile.write(json_string)
        # Create log.json
        with open('log.json', 'w') as outfile:
            data = dict()
            data['token'] = log_token
            data['logfile'] = ''
            data['vehicle'] = 'Cavalier'
            date_captured = datetime.fromtimestamp(reader.start_time * 1e-9).strftime('%Y-%m-%d')
            data['date_captured'] = date_captured
            data['location'] = track_name
            json_string = json.dumps(data)
            outfile.write(json_string)
        # connections = [x for x in reader.connections.values() if (x.topic == '/novatel_top/dyntf_odom' or x.topic == '/luminar_points')]
        # for connection, timestamp, rawdata in reader.messages(connections=connections):
        #     if connection.topic == '/novatel_top/dyntf_odom':
        #         msg = deserialize_cdr(rawdata, connection.msgtype)
        #         print(msg.header)

if __name__=="__main__":
    if len(sys.argv) != 3:
        print("Expecting: [rosbag path] [track name]")
        print("Got: ", end='')
        for arg in sys.argv:
            print(arg, end='')
        exit(1)
    write_scene(sys.argv[1], sys.argv[2])
