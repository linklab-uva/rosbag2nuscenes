#! /usr/bin/python3

import sys
import os
from rosbags.rosbag2 import Reader
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from rosbags.serde import deserialize_cdr
from secrets import token_hex
import json
from datetime import datetime
from tqdm import tqdm

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
        # Create sensor.json
        with open('sensor.json', 'w', encoding='utf-8') as outfile:
            # Combined lidar topic
            lidar_all = dict()
            lidar_all['token'] = token_hex(16)
            lidar_all['channel'] = 'LUMINAR_POINTS'
            lidar_all['modality'] = 'lidar'
            # Front lidar topic
            lidar_front = dict()
            lidar_front['token'] = token_hex(16)
            lidar_front['channel'] = 'LUMINAR_FRONT'
            lidar_front['modality'] = 'lidar'
            # Left lidar topic
            lidar_left = dict()
            lidar_left['token'] = token_hex(16)
            lidar_left['channel'] = 'LUMINAR_LEFT'
            lidar_left['modality'] = 'lidar'
            # Right lidar topic
            lidar_right = dict()
            lidar_right['token'] = token_hex(16)
            lidar_right['channel'] = 'LUMINAR_RIGHT'
            lidar_right['modality'] = 'lidar'

            json.dump([lidar_all, lidar_front, lidar_left, lidar_right], outfile, ensure_ascii=False, indent=4)
        # Create calibrated_sensor.json
        with open('calibrated_sensor.json', 'w', encoding='utf-8') as outfile:
            # TODO: check if combined lidar topic or separated
            data = dict()
            data['token'] = token_hex(16)
            data['sensor_token'] = ''
            data['translation'] = [0.0, 0.0, 0.0]
            data['rotation'] = [0.0, 0.0, 0.0, 1.0]
            data['camera_instrinsic'] = []
            json.dump(data, outfile, ensure_ascii=False, indent=4)
        # Create remaining json files
        connections = [x for x in reader.connections.values() if (x.topic == '/novatel_top/dyntf_odom' or x.topic == '/luminar_points')]
        # TODO: Case where json files already exists
        first_scene, last_scene = '', ''
        prev_scene, next_scene = '', token_hex(16)
        prev_sample, next_sample = '', token_hex(16)
        scene_token = token_hex(16)
        samples = []
        sample_data = []
        ego_poses = []
        sample_json = open('sample.json', 'w', encoding='utf-8')
        sampledata_json = open('sample_data.json', 'w', encoding='utf-8')
        egopose_json = open('ego_pose.json', 'w', encoding='utf-8')
        if not os.path.exists('samples'):
            os.makedirs('samples')
        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
            # Create sample.json
            sample = dict()
            sample_token = next_scene
            sample['token'] = sample_token
            sample['timestamp'] = timestamp
            sample['scene_token'] = scene_token
            sample['prev'] = prev_scene
            prev_scene = next_scene
            next_scene = token_hex(16)
            sample['next'] = next_scene  
            samples.append(sample)
            # Create ego_pose.json
            if connection.topic == '/novatel_top/dyntf_odom':
                msg  = deserialize_cdr(rawdata, connection.msgtype)
                ego_pose = dict()
                ego_pose_token = token_hex(16)
                ego_pose['token'] = ego_pose_token
                ego_pose['timestamp'] = timestamp
                ego_pose['rotation'] = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
                ego_pose['translation'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
                ego_poses.append(ego_pose)

            # Create sample_data.json
            if connection.topic == '/luminar_points':
                ros2_msg  = deserialize_cdr(rawdata, connection.msgtype)
                msg = PointCloud2()
                msg.data =ros2_msg.data
                msg.fields = ros2_msg.fields
                msg.header = ros2_msg.header
                msg.height = ros2_msg.height
                msg.is_bigendian = ros2_msg.is_bigendian
                msg.is_dense = ros2_msg.is_dense
                msg.point_step = ros2_msg.point_step
                msg.row_step = ros2_msg.row_step
                msg.width = ros2_msg.width
                data = dict()
                data_token = token_hex(16)
                data['token'] = data_token
                data['sample_token'] = prev_scene
                data['ego_pose_token'] = ego_pose_token
                data['calibrated_sensor_token'] = 'TODO'
                
                points_list = []

                for point in pc2.read_points(msg, skip_nans=True):
                    points_list.append([point[0], point[1], point[2], point[3]])

                pcl_data = pcl.PointCloud_PointXYZRGB()
                pcl_data.from_list(points_list)

                filename = "samples/{0}.pcd".format(data_token)
                pcl.save(pcl_data, filename)
                data['filename'] = filename
                data['fileformat'] = 'pcd'
                data['is_key_frame'] = 'TODO'
                data['height'] = 0
                data['width'] = 0
                data['timestamp'] = timestamp
                data['prev'] = prev_sample
                prev_sample = next_sample
                next_sample = token_hex(16)
                data['next'] = next_sample
                sample_data.append(data)


        samples[-1]['next'] = ''
        sample_data[-1]['next'] = ''
        json.dump(samples, sample_json, ensure_ascii=False, indent=4)
        json.dump(ego_poses, egopose_json, ensure_ascii=False, indent=4)
        json.dump(sample_data, sampledata_json, ensure_ascii=False, indent=4)
        last_scene = prev_scene
        sample_json.close()
        egopose_json.close()
        sampledata_json.close()
                

                
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
