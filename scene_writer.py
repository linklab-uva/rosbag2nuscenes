#! /usr/bin/python3

import sys
import os
import rclpy
from typing import Union
import numpy as np
import rosbag2_py
from sensor_msgs_py.point_cloud2 import read_points
import sensor_msgs
import tf2_msgs
import delphi_mrr_msgs
import nav_msgs
from secrets import token_hex
import json
from datetime import datetime
from tqdm import tqdm
import itertools
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

allowable_tracks = {'LVMS', 'IMS'}
lidar_topics = {'/luminar_left_points', '/luminar_right_points', '/luminar_front_points'}
radar_topics = {}
camera_topics = {'/camera/front_left/image/compressed', '/camera/front_left_center/image/compressed', '/camera/front_left_center/image/compressed', '/camera/front_left_center/image/compressed', '/camera/rear_right/image/compressed'}
odom_topic = '/novatel_top/odom'

def get_rosbag_options(path, serialization_format="cdr", storage_id="sqlite3"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def open_bagfile(filepath: str, serialization_format="cdr", storage_id="sqlite3"):
    storage_options, converter_options = get_rosbag_options(filepath, serialization_format=serialization_format, storage_id=storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    # Create maps for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    topic_metadata_map = {topic_types[i].name: topic_types[i] for i in range(len(topic_types))}
    return topic_types, type_map, topic_metadata_map, reader

def headerKey( tup ):
    msg : Union[ tf2_msgs.msg.TFMessage, nav_msgs.msg.Odometry, sensor_msgs.msg.PointCloud, delphi_mrr_msgs.msg.Detection, sensor_msgs.msg.CompressedImage, sensor_msgs.msg.CameraInfo] = tup[1]
    return rclpy.time.Time.from_msg(msg.header.stamp)

def write_scene(argdict):
    bag_dir = os.path.normpath(os.path.abspath(argdict["bag_in"]))
    rosbag_file = argdict["bag_in"]
    metadatafile : str = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.isfile(metadatafile):
        raise ValueError("Metadata file %s does not exist. Are you sure %s is a valid rosbag?" % (metadatafile, bag_dir))
    with open(metadatafile, "r") as f:
        metadata_dict : dict = yaml.load(f, Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
    topic_types, type_map, topic_metadata_map, reader = open_bagfile(bag_dir)
    topic_count_dict = {entry["topic_metadata"]["name"] : entry["message_count"] for entry in metadata_dict["topics_with_message_count"]}
    topic_counts = np.array( list(topic_count_dict.values()) ) 
    total_msgs = np.sum( topic_counts )
    msg_dict = {key : [] for key in topic_count_dict.keys()}

    for idx in tqdm(iterable=range(total_msgs)):
        if(reader.has_next()):
            (topic, data, t) = reader.read_next()
            if topic == odom_topic or topic == '/tf_static':
                msg_type = type_map[topic]
                msg_type_full = get_message(msg_type)
                msg = deserialize_message(data, msg_type_full)
                msg_dict[topic].append((t, msg))
    track_num = ''
    while (track_num != '1' and track_num != '2'):
        track_num = input("Possible tracks\n\t[1] IMS\n\t[2] LVMS\nSelect track number: ")
    if track_num == '1':
        track_name = "IMS"
    else:
        track_name = "LVMS"
    description = input("Enter a short description of the bag file scenario: ")

    # Create log.json
    log_token = token_hex(16)
    with open('log.json', 'a', encoding='utf-8') as outfile:
        data = dict()
        data['token'] = log_token
        if rosbag_file[-1] == '/':
            rosbag_file = rosbag_file[:-1]
        data['logfile'] = rosbag_file.split('/')[-1]
        data['vehicle'] = 'TODO'
        date_captured = datetime.fromtimestamp(metadata_dict["starting_time"]["nanoseconds_since_epoch"] * 1e-9).strftime('%Y-%m-%d')
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
            # Front lidar
            lidar_front = dict()
            front_token = token_hex(16)
            lidar_front['token'] = front_token
            lidar_front['channel'] = 'LIDAR_FRONT'
            lidar_front['modality'] = 'lidar'
            os.makedirs('samples/LIDAR_FRONT')
            os.makedirs('sweeps/LIDAR_FRONT')
            # Left lidar
            lidar_left = dict()
            left_token = token_hex(16)
            lidar_left['token'] = left_token
            lidar_left['channel'] = 'LIDAR_LEFT'
            lidar_left['modality'] = 'lidar'
            os.makedirs('samples/LIDAR_LEFT')
            os.makedirs('sweeps/LIDAR_LEFT')
            # Right lidar
            lidar_right = dict()
            right_token = token_hex(16)
            lidar_right['token'] = right_token
            lidar_right['channel'] = 'LIDAR_RIGHT'
            lidar_right['modality'] = 'lidar'
            os.makedirs('samples/LIDAR_RIGHT')
            os.makedirs('sweeps/LIDAR_RIGHT')
            # Combined lidar
            lidar_all = dict()
            all_token = token_hex(16)
            lidar_all['token'] = all_token
            lidar_all['channel'] = 'LIDAR_COMBINED'
            lidar_all['modality'] = 'lidar'
            os.makedirs('samples/LIDAR_COMBINED')
            os.makedirs('sweeps/LIDAR_COMBINED')

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
        for timestamp, msg in msg_dict['/tf_static']:
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
    ego_pose_queue = []

    print("Serializing ego poses")
    for timestamp, msg in tqdm(msg_dict['/novatel_top/odom']):
        # Create ego_pose.json
        ego_pose = dict()
        ego_pose_token = token_hex(16)
        ego_pose_queue.append((ego_pose_token, timestamp))
        ego_pose['token'] = ego_pose_token
        ego_pose['timestamp'] = timestamp * 1e-9
        ego_pose['rotation'] = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        ego_pose['translation'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        ego_poses.append(ego_pose)

    print("Serializing sensor data")
    previous_sampled_timestamp = None
    nbr_samples = 0
    previous_front_token, previous_left_token, previous_right_token, previous_all_token = '', '', '', ''
    next_front_token, next_left_token, next_right_token, next_all_token = token_hex(16), token_hex(16), token_hex(16), token_hex(16)
    sensors_added = set()
    topic_types, type_map, topic_metadata_map, reader = open_bagfile(bag_dir)
    for idx in tqdm(iterable=range(total_msgs)):
        if(reader.has_next()):
            (topic, data, timestamp) = reader.read_next()
            if topic in lidar_topics:
                msg_type = type_map[topic]
                msg_type_full = get_message(msg_type)
                msg = deserialize_message(data, msg_type_full)
                if previous_sampled_timestamp is None or (datetime.fromtimestamp(timestamp * 1e-9) - datetime.fromtimestamp(previous_sampled_timestamp * 1e-9)).total_seconds() > 0.5:
                    previous_sampled_timestamp = timestamp
                    nbr_samples += 1
                    # Create sample.json
                    sample = dict()
                    sample_token = next_sample_token
                    sample['token'] = sample_token
                    sample['timestamp'] = timestamp * 1e-9
                    sample['scene_token'] = scene_token
                    sample['prev'] = prev_sample_token
                    prev_sample_token = next_sample_token
                    next_sample_token = token_hex(16)
                    sample['next'] = next_sample_token
                    samples.append(sample)
                    sensors_added.clear()

                # Create sample_data.json
                data = dict()
                if topic == '/luminar_points':
                    sensor_token = all_calibrated_sensor_token
                    sensor_name = 'LIDAR_COMBINED'
                    prev_data_token = previous_all_token
                    data_token = next_all_token
                    next_all_token = token_hex(16)
                    previous_all_token = data_token
                    next_data_token = next_all_token
                elif topic == '/luminar_front_points':
                    sensor_token = front_calibrated_sensor_token
                    sensor_name = 'LIDAR_FRONT'
                    prev_data_token = previous_front_token
                    data_token = next_front_token
                    next_front_token = token_hex(16)
                    previous_front_token = data_token
                    next_data_token = next_front_token
                elif topic == '/luminar_left_points':
                    sensor_token = left_calibrated_sensor_token
                    sensor_name = 'LIDAR_LEFT'
                    prev_data_token = previous_left_token
                    data_token = next_left_token
                    next_left_token = token_hex(16)
                    previous_left_token = data_token
                    next_data_token = next_left_token
                elif topic == '/luminar_right_points':
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
                # Find closest ego pose
                previous_time_difference = np.inf
                previous_loc = 0
                for i in range(previous_loc, len(ego_pose_queue)):
                    time_difference = abs((datetime.fromtimestamp(timestamp * 1e-9) - datetime.fromtimestamp(ego_pose_queue[i][1] * 1e-9)).total_seconds())
                    if time_difference < previous_time_difference:
                        previous_time_difference = time_difference
                    else:
                        ego_pose_token = ego_pose_queue[i-1]
                        previous_loc = i - 1
                        break
                data['ego_pose_token'] = ego_pose_token[0]
                saved_points = np.zeros((msg.width, 5))
                point_num = 0
                for point in read_points(msg, skip_nans=True):
                    saved_points[point_num,0]=point[0]
                    saved_points[point_num,1]=point[1]
                    saved_points[point_num,2]=point[2]
                    saved_points[point_num,3]=point[3]
                    point_num += 1
                if sensor_name not in sensors_added:
                    filename = "samples/{0}/{1}__{0}__{2}.pcd.bin".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    sensors_added.add(sensor_name)
                    is_key_frame = True
                    if first_scene == '':
                        first_scene = sample_token
                else:
                    filename = "sweeps/{0}/{1}__{0}__{2}.pcd.bin".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    is_key_frame = False
                with open(filename, 'wb') as pcd_file:
                    saved_points.astype('float32').tofile(pcd_file)
                data['filename'] = filename
                data['fileformat'] = 'pcd'
                data['is_key_frame'] = is_key_frame
                data['height'] = 0
                data['width'] = 0
                data['timestamp'] = timestamp * 1e-9
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
    import argparse, argcomplete
    parser = argparse.ArgumentParser(description="Label point clouds with bounding boxes.")
    parser.add_argument("bag_in", type=str, help="Bag to load")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    argdict : dict = vars(args)
    write_scene(argdict)
