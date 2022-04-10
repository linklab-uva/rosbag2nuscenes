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

def write_scene(rosbag_file):
    track_num = ''
    while (track_num != '1' and track_num != '2'):
        track_num = input("Possible tracks\n\t[1] IMS\n\t[2] LVMS\nSelect track number: ")
    if track_num == '1':
        track_name = "IMS"
    else:
        track_name = "LVMS"
    description = input("Enter a short description of the bag file scenario: ")
    with Reader(rosbag_file) as reader:
        # Create log.json
        log_token = token_hex(16)
        with open('log.json', 'a', encoding='utf-8') as outfile:
            data = dict()
            data['token'] = log_token
            if rosbag_file[-1] == '/':
                rosbag_file = rosbag_file[:-1]
            data['logfile'] = rosbag_file.split('/')[-1]
            data['vehicle'] = 'Cavalier'
            date_captured = datetime.fromtimestamp(reader.start_time * 1e-9).strftime('%Y-%m-%d')
            data['date_captured'] = date_captured
            data['location'] = track_name
            json.dump([data], outfile, indent=4)
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
                json.dump(new_json, outfile, indent=4)
        else:
        # Case where map.json does not exist
            with open('map.json', 'w', encoding='utf-8') as outfile:
                ims_data = dict()
                ims_data['token'] = token_hex(16)
                ims_data['log_tokens'] = []
                if track_name == 'IMS':
                    ims_data['log_tokens'].append(log_token)
                ims_data['category'] = 'IMS'
                ims_data['filename'] = '' 
                lvms_data = dict()
                lvms_data['token'] = token_hex(16)
                lvms_data['log_tokens'] = []
                if track_name == 'LVMS':
                    lvms_data['log_tokens'].append(log_token)
                lvms_data['category'] = 'LVMS'
                lvms_data['filename'] = ''
                json.dump([ims_data, lvms_data], outfile, indent=4)
        # Create sensor.json
        if not os.path.exists('sensor.json'):
            with open('sensor.json', 'w', encoding='utf-8') as outfile:
                # Front lidar topic
                lidar_front = dict()
                front_token = token_hex(16)
                lidar_front['token'] = front_token
                lidar_front['channel'] = 'LIDAR_FRONT'
                lidar_front['modality'] = 'lidar'
                # Left lidar topic
                lidar_left = dict()
                left_token = token_hex(16)
                lidar_left['token'] = left_token
                lidar_left['channel'] = 'LIDAR_LEFT'
                lidar_left['modality'] = 'lidar'
                # Right lidar topic
                lidar_right = dict()
                right_token = token_hex(16)
                lidar_right['token'] = right_token
                lidar_right['channel'] = 'LIDAR_RIGHT'
                lidar_right['modality'] = 'lidar'
                # Combined lidar topic
                lidar_all = dict()
                all_token = token_hex(16)
                lidar_all['token'] = all_token
                lidar_all['channel'] = 'LIDAR_COMBINED'
                lidar_all['modality'] = 'lidar'

                json.dump([lidar_front, lidar_left, lidar_right, lidar_all], outfile, indent=4)
        else:
            with open('sensor.json', 'r', encoding='utf-8') as readfile:
                sensors = json.load(readfile)
                for sensor in sensors:
                    if sensor['channel'] == 'LIDAR_FRONT':
                        front_token = sensor['token']
                    elif sensor['channel'] == 'LIDAR_LEFT':
                        left_token = sensor['token']
                    elif sensor['channel'] == 'LIDAR_FRONT':
                        right_token = sensor['token']
                    elif sensor['channel'] == 'LIDAR_COMBINED':
                        all_token = sensor['token']
        # Create calibrated_sensor.json
        with open('calibrated_sensor.json', 'a', encoding='utf-8') as outfile:
            frames_received = []
            for connection, timestamp, rawdata in reader.messages(connections=[x for x in reader.connections.values() if (x.topic == '/tf_static')]):
                msg = deserialize_cdr(rawdata, connection.msgtype)
                new_json = []
                for transform in msg.transforms:
                    if transform.child_frame_id == 'luminar_front' and 'luminar_front' not in frames_received:
                        data = dict()
                        front_calibrated_sensor_token = token_hex(16)
                        data['token'] = front_calibrated_sensor_token
                        data['sensor_token'] = front_token
                        data['translation'] = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                        data['rotation'] = [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]
                        data['camera_instrinsic'] = []
                        frames_received.append('luminar_front')
                        
                    elif transform.child_frame_id == 'luminar_left' and 'luminar_left' not in frames_received:
                        data = dict()
                        left_calibrated_sensor_token = token_hex(16)
                        data['token'] = left_calibrated_sensor_token
                        data['sensor_token'] = left_token
                        data['translation'] = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                        data['rotation'] = [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]
                        data['camera_instrinsic'] = []
                        frames_received.append('luminar_left')
                    elif transform.child_frame_id == 'luminar_right' and 'luminar_right' not in frames_received:
                        data = dict()
                        right_calibrated_sensor_token = token_hex(16)
                        data['token'] = right_calibrated_sensor_token
                        data['sensor_token'] = right_token
                        data['translation'] = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                        data['rotation'] = [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]
                        data['camera_instrinsic'] = []
                        frames_received.append('luminar_right')
                    elif 'luminar_all' not in frames_received:
                        data = dict()
                        all_calibrated_sensor_token = token_hex(16)
                        data['token'] = all_calibrated_sensor_token
                        data['sensor_token'] = all_token
                        data['translation'] = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                        data['rotation'] = [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]
                        data['camera_instrinsic'] = []
                        frames_received.append('luminar_all')
                    else:
                        continue
                    new_json.append(data)
                if len(frames_received) == 4:
                    break
            json.dump(new_json, outfile, indent=4)
        # Create remaining json files
        first_scene = ''
        prev_sample_token, next_sample_token = '', token_hex(16)
        scene_token = token_hex(16)
        samples = []
        sample_data = []
        ego_poses = []
        sample_json = open('sample.json', 'a', encoding='utf-8')
        sampledata_json = open('sample_data.json', 'a', encoding='utf-8')
        egopose_json = open('ego_pose.json', 'a', encoding='utf-8')
        if not os.path.exists('samples'):
            os.makedirs('samples')
        if not os.path.exists('sweeps'):
            os.makedirs('sweeps')
        connections = [x for x in reader.connections.values() if (x.topic == '/novatel_top/dyntf_odom' or x.topic == '/dyntf_odom')]
        ego_pose_queue = []
        print("Serializing ego poses")
        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
            # Create ego_pose.json
            msg  = deserialize_cdr(rawdata, connection.msgtype)
            ego_pose = dict()
            ego_pose_token = token_hex(16)
            ego_pose_queue.append((ego_pose_token, timestamp))
            ego_pose['token'] = ego_pose_token
            ego_pose['timestamp'] = timestamp
            ego_pose['rotation'] = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            ego_pose['translation'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            ego_poses.append(ego_pose)
        print("Serializing lidar data")
        previous_sampled_timestamp = None
        nbr_samples = 0
        connections = [x for x in reader.connections.values() if (x.topic == '/luminar_points' or x.topic == '/luminar_front_points' or x.topic == '/luminar_left_points' or x.topic == '/luminar_right_points')]
        previous_front_token, previous_left_token, previous_right_token, previous_all_token = '', '', '', ''
        next_front_token, next_left_token, next_right_token, next_all_token = token_hex(16), token_hex(16), token_hex(16), token_hex(16)
        sensors_added = set()
        for connection, timestamp, rawdata in tqdm(reader.messages(connections=connections)):
            if previous_sampled_timestamp is None or (datetime.fromtimestamp(timestamp * 1e-9) - datetime.fromtimestamp(previous_sampled_timestamp * 1e-9)).total_seconds() > 0.5:
                previous_sampled_timestamp = timestamp
                nbr_samples += 1
                # Create sample.json
                sample = dict()
                sample_token = next_sample_token
                sample['token'] = sample_token
                sample['timestamp'] = timestamp
                sample['scene_token'] = scene_token
                sample['prev'] = prev_sample_token
                prev_sample_token = next_sample_token
                next_sample_token = token_hex(16)
                sample['next'] = next_sample_token
                samples.append(sample)
                sensors_added.clear()

            # Create sample_data.json
            ros2_msg  = deserialize_cdr(rawdata, connection.msgtype)
            msg = PointCloud2()
            msg.data = ros2_msg.data
            msg.fields = ros2_msg.fields
            msg.header = ros2_msg.header
            msg.height = ros2_msg.height
            msg.is_bigendian = ros2_msg.is_bigendian
            msg.is_dense = ros2_msg.is_dense
            msg.point_step = ros2_msg.point_step
            msg.row_step = ros2_msg.row_step
            msg.width = ros2_msg.width
            data = dict()
            if connection.topic == '/luminar_points':
                sensor_token = all_calibrated_sensor_token
                sensor_name = 'LIDAR_COMBINED'
                prev_data_token = previous_all_token
                data_token = next_all_token
                next_all_token = token_hex(16)
                previous_all_token = data_token
                next_data_token = next_all_token
            elif connection.topic == '/luminar_front_points':
                sensor_token = front_calibrated_sensor_token
                sensor_name = 'LIDAR_FRONT'
                prev_data_token = previous_front_token
                data_token = next_front_token
                next_front_token = token_hex(16)
                previous_front_token = data_token
                next_data_token = next_front_token
            elif connection.topic == '/luminar_left_points':
                sensor_token = left_calibrated_sensor_token
                sensor_name = 'LIDAR_LEFT'
                prev_data_token = previous_left_token
                data_token = next_left_token
                next_left_token = token_hex(16)
                previous_left_token = data_token
                next_data_token = next_left_token
            else:
                sensor_token = right_calibrated_sensor_token
                sensor_name = 'LIDAR_RIGHT'
                prev_data_token = previous_right_token
                data_token = next_right_token
                next_right_token = token_hex(16)
                previous_right_token = data_token
                next_data_token = next_right_token
            data['token'] = data_token
            data['sample_token'] = sample_token
            data['calibrated_sensor_token'] = sensor_token
            # Find closest ego pose (TODO: optimize)
            previous_time_difference = np.inf
            for i in range(len(ego_pose_queue)):
                time_difference = abs((datetime.fromtimestamp(timestamp * 1e-9) - datetime.fromtimestamp(ego_pose_queue[i][1] * 1e-9)).total_seconds())
                if time_difference < previous_time_difference:
                    previous_time_difference = time_difference
                else:
                    ego_pose_token = ego_pose_queue[i-1]
                    break
            data['ego_pose_token'] = ego_pose_token[0]
            points_list = []
            for point in pc2.read_points(msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2], point[3]])

            if sensor_name not in sensors_added:
                if not os.path.exists('samples/{0}'.format(sensor_name)):
                    os.makedirs('samples/{0}'.format(sensor_name))
                filename = "samples/{0}/{1}__{0}__{2}.pcd.bin".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                sensors_added.add(sensor_name)
                is_key_frame = True
                if first_scene == '':
                    first_scene = sample_token
            else:
                if not os.path.exists('sweeps/{0}'.format(sensor_name)):
                    os.makedirs('sweeps/{0}'.format(sensor_name))
                filename = "sweeps/{0}/{1}__{0}__{2}.pcd.bin".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                is_key_frame = False
            with open(filename, 'wb') as pcd_file:
                for point in points_list:
                    pcd_file.write(b'{0} {1} {2} {3}'.format(point[0], point[1], point[2], point[3]))
            data['filename'] = filename
            data['fileformat'] = 'pcd'
            data['is_key_frame'] = is_key_frame
            data['height'] = 0
            data['width'] = 0
            data['timestamp'] = timestamp
            data['prev'] = prev_data_token
            data['next'] = next_data_token
            sample_data.append(data)

        samples[-1]['next'] = ''
        sample_data[-1]['next'] = ''
        json.dump(samples, sample_json, indent=4)
        json.dump(ego_poses, egopose_json, indent=4)
        json.dump(sample_data, sampledata_json, indent=4)
        sample_json.close()
        egopose_json.close()
        sampledata_json.close()
                
        # Create scene.json
        with open('scene.json', 'w', encoding='utf-8') as outfile:
            data = dict()
            data['token'] = scene_token
            data['log_token'] = log_token
            data['nbr_samples'] = nbr_samples
            data['first_sample_token'] = first_scene
            data['last_sample_token'] = sample_token
            if rosbag_file[-1] == '/':
                rosbag_file = rosbag_file[:-1]
            data['name'] = rosbag_file.split('/')[-1]
            data['description'] = description
            json.dump([data], outfile, indent=4)

if __name__=="__main__":
    if len(sys.argv) != 2:
        print("Expecting: ./scene_writer.py [rosbag path]")
        print("Got: ", end='')
        for arg in sys.argv:
            print(arg, end='')
        print('\n')
        exit(1)
    write_scene(sys.argv[1])
