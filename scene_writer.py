#! /usr/bin/env python

import sys
import os
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
        # Create log.json
        log_token = token_hex(16)
        with open('log.json', 'w', encoding='utf-8') as outfile:
            data = dict()
            data['token'] = log_token
            data['logfile'] = ''
            data['vehicle'] = 'Cavalier'
            date_captured = datetime.fromtimestamp(reader.start_time * 1e-9).strftime('%Y-%m-%d')
            data['date_captured'] = date_captured
            data['location'] = track_name
            json.dump([data], outfile, ensure_ascii=False, indent=4)
        # Create map.json
        # Case where map file exists
        if os.path.exists('map.json'):
            with open('map.json', 'r') as outfile:
                data = json.load(outfile)
            new_json = []
            for object in data:
                if object['category'] == track_name:
                    object['log_tokens'].append(log_token)
                new_json.append(object)
            with open('map.json', 'w') as outfile:
                json.dump(new_json, outfile, ensure_ascii=False, indent=4)
        else:
        # Case where map.json does not exist
            with open('map.json', 'w', encoding='utf-8') as outfile:
                ims_data = dict()
                ims_data['token'] = token_hex(16)
                ims_data['log_tokens'] = []
                if track_name == 'IMS':
                    ims_data['log_tokens'].append(log_token)
                ims_data['category'] = 'IMS'
                ims_data['filename'] = '' # TODO: double check we aren't including map mask
                lvms_data = dict()
                lvms_data['token'] = token_hex(16)
                lvms_data['log_tokens'] = []
                if track_name == 'LVMS':
                    lvms_data['log_tokens'].append(log_token)
                lvms_data['category'] = 'LVMS'
                lvms_data['filename'] = '' # TODO: double check we aren't including map mask
                json.dump([ims_data, lvms_data], outfile, ensure_ascii=False, indent=4)
        connections = [x for x in reader.connections.values() if (x.topic == '/novatel_top/dyntf_odom' or x.topic == '/luminar_points')]
        # Create sample.json
        # TODO: Case where sample.json already exists
        first_scene, last_scene = '', ''
        prev_scene, next_scene = '', ''
        scene_token = token_hex(16)
        samples = []
        with open('sample.json', 'w', encoding='utf-8') as outfile:
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                data = dict()
                data['timestamp'] = timestamp
                data['scene_token'] = scene_token
                data['prev'] = prev_scene
                prev_scene = next_scene
                next_scene = token_hex(16)
                data['next'] = next_scene  
                if first_scene == '':
                    prev_scene = token_hex(16)
                    first_scene = prev_scene
                samples.append(data)
            samples[-1]['next'] = ''
            json.dump(samples, outfile, ensure_ascii=False, indent=4)
        last_scene = prev_scene
                

                
                # if connection.topic == '/novatel_top/dyntf_odom':
                #     msg = deserialize_cdr(rawdata, connection.msgtype)
                #     print(msg.header)
        # Create scene.json
        with open('scene.json', 'w', encoding='utf-8') as outfile:
            data = dict()
            data['token'] = scene_token
            data['log_token'] = log_token
            connection = [x for x in reader.connections.values() if x.topic == '/novatel_top/dyntf_odom'][0]
            data['nbr_samples'] = connection.count
            first_scene, last_scene = token_hex(16), token_hex(16)
            data['first_sample_token'] = first_scene
            data['last_sample_token'] = last_scene
            data['name'] = 'scene-0001'  # TODO: increase global variable to create name
            data['description'] = track_name
            json.dump([data], outfile, ensure_ascii=False, indent=4)

if __name__=="__main__":
    if len(sys.argv) != 3:
        print("Expecting: [rosbag path] [track name]")
        print("Got: ", end='')
        for arg in sys.argv:
            print(arg, end='')
        exit(1)
    write_scene(sys.argv[1], sys.argv[2])
