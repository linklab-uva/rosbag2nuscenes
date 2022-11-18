#! /usr/bin/env python3

from multiprocessing.sharedctypes import Value
import sys
import os
import rclpy
from typing import Union
import numpy as np
import rosbag2_py
from sensor_msgs_py.point_cloud2 import read_points
import sensor_msgs
import tf2_msgs
import nav_msgs
from secrets import token_hex
import json
from datetime import datetime
from tqdm import tqdm
import itertools
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import cv2
from cv_bridge import CvBridge
import math

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
    msg : Union[ tf2_msgs.msg.TFMessage, nav_msgs.msg.Odometry, sensor_msgs.msg.PointCloud, delphi_esr_msgs.msg.EsrTrack, sensor_msgs.msg.CompressedImage, sensor_msgs.msg.CameraInfo] = tup[1]
    return rclpy.time.Time.from_msg(msg.header.stamp)

def write_scene(argdict):
    bag_dir = os.path.normpath(os.path.abspath(argdict["bag_in"]))
    rosbag_file = argdict["bag_in"]
    metadatafile : str = os.path.join(bag_dir, "metadata.yaml")
    param_file = os.path.normpath(os.path.abspath(argdict["param_file"]))
    # Read in param file
    if not os.path.isfile(param_file):
        raise ValueError("Param file %s does not exist" % param_file)
    with open(param_file, "r") as f:
        param_dict : dict = yaml.load(f, Loader=yaml.SafeLoader)
    # Extract info from param file
    track_name = param_dict["BAG_INFO"]["TRACK"]
    lidar_topics = dict()
    radar_topics = dict()
    camera_topics = dict()
    camera_calibs = dict()
    for sensor_name in param_dict["SENSOR_INFO"]:
        modality = sensor_name.split('_')[0]
        if modality == "LIDAR":
            if param_dict["SENSOR_INFO"][sensor_name]["TOPIC"]:
                lidar_topics[param_dict["SENSOR_INFO"][sensor_name]["TOPIC"]] = sensor_name
        elif modality == "RADAR":
            if param_dict["SENSOR_INFO"][sensor_name]["TOPIC"]:
                radar_topics[param_dict["SENSOR_INFO"][sensor_name]["TOPIC"]] = sensor_name
        elif modality == "CAMERA":
            if param_dict["SENSOR_INFO"][sensor_name]["TOPIC"]:
                camera_topics[param_dict["SENSOR_INFO"][sensor_name]["TOPIC"]] = sensor_name
                camera_calibs[param_dict["SENSOR_INFO"][sensor_name]["FRAME"]] = param_dict["SENSOR_INFO"][sensor_name]["CALIB"]
        else:
            raise ValueError("Invalid sensor %s in %s. Ensure sensor is of type LIDAR, RADAR, or CAMERA and is named [SENSOR TYPE]_[SENSOR LOCATION]" % (sensor_name, param_file))
    # Extract metadata from bag directory
    if not os.path.isfile(metadatafile):
        raise ValueError("Metadata file %s does not exist. Are you sure %s is a valid rosbag?" % (metadatafile, bag_dir))
    with open(metadatafile, "r") as f:
        metadata_dict : dict = yaml.load(f, Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
    topic_types, type_map, topic_metadata_map, reader = open_bagfile(bag_dir)
    topic_count_dict = {entry["topic_metadata"]["name"] : entry["message_count"] for entry in metadata_dict["topics_with_message_count"]}
    topic_counts = np.array( list(topic_count_dict.values()) ) 
    total_msgs = np.sum( topic_counts )
    msg_dict = {key : [] for key in topic_count_dict.keys()}
    # Store all odom and tf static messages
    for idx in tqdm(iterable=range(total_msgs)):
        if(reader.has_next()):
            (topic, data, t) = reader.read_next()
            if topic == param_dict["BAG_INFO"]["ODOM_TOPIC"] or topic == '/tf_static' or topic in camera_calibs.values():
                msg_type = type_map[topic]
                msg_type_full = get_message(msg_type)
                msg = deserialize_message(data, msg_type_full)
                msg_dict[topic].append((t, msg))

    if not os.path.isdir('v1.0-mini'):
        os.mkdir('v1.0-mini')
    # Create log.json
    log_token = token_hex(16)
    if os.path.exists('v1.0-mini/log.json'):
        with open('v1.0-mini/log.json', 'r') as f:
            logs = json.load(f)
    else:
        logs = []
    new_log = dict()
    new_log['token'] = log_token
    if rosbag_file[-1] == '/':
        rosbag_file = rosbag_file[:-1]
    new_log['logfile'] = rosbag_file.split('/')[-1]
    new_log['vehicle'] = param_dict["BAG_INFO"]["TEAM"]
    new_log['date_captured'] = datetime.fromtimestamp(metadata_dict["starting_time"]["nanoseconds_since_epoch"] * 1e-9).strftime('%Y-%m-%d')
    new_log['location'] = track_name
    logs.append(new_log)
    with open('v1.0-mini/log.json', 'w') as f:
        json.dump(logs, f, indent=4)
    # Create map.json
    # If map file already exists, add log_token to existing map 
    if os.path.exists('v1.0-mini/map.json'):
        with open('v1.0-mini/map.json', 'r') as f:
            maps = json.load(f)
        new_json = []
        map_exists = False
        for map in maps:
            if map['category'] == track_name:
                map['log_tokens'].append(log_token)
                map_exists = True
            new_json.append(map)
        if not map_exists:
            new_map = dict()
            new_map['token'] = token_hex(16)
            new_map['log_tokens'] = [log_token]
            new_map['category'] = track_name
            new_map['filename'] = ''
            new_json.append(new_map)
        with open('v1.0-mini/map.json', 'w') as f:
            json.dump(new_json, f, indent=4)
    else:
    # Case where map.json does not exist
        with open('v1.0-mini/map.json', 'w') as f:
            map = dict()
            map['token'] = token_hex(16)
            map['log_tokens'] = [log_token]
            map['category'] = track_name
            map['filename'] = '' 
            json.dump([map], f, indent=4)
    # Create sensor.json
    sensor_token_dict = dict()
    if not os.path.exists('v1.0-mini/sensor.json'):
        sensor_configs = []
        for channel in param_dict['SENSOR_INFO']:
            sensor_config = dict()
            sensor_token = token_hex(16)
            # Store sensor token for use in calibrated_sensor.json
            sensor_token_dict[param_dict['SENSOR_INFO'][channel]['FRAME']] = sensor_token
            sensor_config['token'] = sensor_token
            sensor_config['channel'] = channel
            sensor_config['modality'] = channel.split('_')[0].lower()
            os.makedirs('samples/%s' % channel)
            os.makedirs('sweeps/%s' % channel)
            sensor_configs.append(sensor_config)
        with open('v1.0-mini/sensor.json', 'w') as f:
            json.dump(sensor_configs, f, indent=4)
    # If sensor.json exists, load existing tokens to param_dict
    else:
        with open('v1.0-mini/sensor.json', 'r') as f:
            sensors = json.load(f)
            for sensor in sensors:
                sensor_token_dict[param_dict['SENSOR_INFO'][sensor['channel']]['FRAME']] = sensor['token']
                
    # Create calibrated_sensor.json
    if os.path.exists('v1.0-mini/calibrated_sensor.json'):
        with open('v1.0-mini/calibrated_sensor.json', 'r') as f:
            calibrated_sensors = json.load(f)
    else:
        calibrated_sensors = []
    frames_received = []
    for timestamp, msg in msg_dict['/tf_static']:
        for transform in msg.transforms:
            if transform.child_frame_id in sensor_token_dict.keys() and transform.child_frame_id not in frames_received:
                calibrated_sensor_data = dict()
                calibrated_sensor_token = token_hex(16)
                calibrated_sensor_data['token'] = calibrated_sensor_token
                calibrated_sensor_data['sensor_token'] = sensor_token_dict[transform.child_frame_id]
                calibrated_sensor_data['translation'] = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                calibrated_sensor_data['rotation'] = [transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z]
                if transform.child_frame_id in camera_calibs:
                    calibrated_sensor_data['camera_intrinsic'] = np.reshape(msg_dict[camera_calibs[transform.child_frame_id]][0][1].k, (3,3)).tolist()
                else:
                    calibrated_sensor_data['camera_intrinsic'] = []
                # Store calibrated sensor token for use in creating sample_data.json
                sensor_token_dict[transform.child_frame_id] = calibrated_sensor_token
                # Mark frame as processed
                frames_received.append(transform.child_frame_id)
                calibrated_sensors.append(calibrated_sensor_data)
        if len(frames_received) == len(lidar_topics) + len(radar_topics) + len(camera_topics):
            break
    with open('v1.0-mini/calibrated_sensor.json', 'w') as f:
        json.dump(calibrated_sensors, f, indent=4)
    
    # Create remaining json files
    first_sample = ''
    prev_sample_token, next_sample_token = '', token_hex(16)
    scene_token = token_hex(16)
    if os.path.exists('v1.0-mini/sample.json'):
        with open('v1.0-mini/sample.json', 'r') as f:
            samples = json.load(f)
    else:
        samples = []
    if os.path.exists('v1.0-mini/sample_data.json'):
        with open('v1.0-mini/sample_data.json', 'r') as f:
            sample_data = json.load(f)
    else:
        sample_data = []
    if os.path.exists('v1.0-mini/ego_pose.json'):
        with open('v1.0-mini/ego_pose.json', 'r') as f:
            ego_poses = json.load(f)
    else:
        ego_poses = []
    ego_pose_queue = []

    print("Extracting odometry data")
    for timestamp, msg in tqdm(msg_dict[param_dict["BAG_INFO"]["ODOM_TOPIC"]]):
        # Create ego_pose.json
        ego_pose = dict()
        ego_pose_token = token_hex(16)
        ego_pose_queue.append((ego_pose_token, timestamp, np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])))
        ego_pose['token'] = ego_pose_token
        ego_pose['timestamp'] = timestamp * 1e-9
        ego_pose['rotation'] = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        ego_pose['translation'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        ego_poses.append(ego_pose)

    print("Extracting sensor data")
    previous_sampled_timestamp = None
    nbr_samples = 0
    previous_loc = 0
    data_token_dict = dict()
    img_converter = CvBridge()
    for topic in list(lidar_topics.keys()) + list(radar_topics.keys()) + list(camera_topics.keys()):
        data_token_dict[topic] = ['', token_hex(16)]
    sensors_added = set()
    topic_types, type_map, topic_metadata_map, reader = open_bagfile(bag_dir)
    for idx in tqdm(iterable=range(total_msgs)):
        if(reader.has_next()):
            (topic, data, timestamp) = reader.read_next()
            if not (topic in lidar_topics or topic in radar_topics or topic in camera_topics):
                continue
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
            sensor_token = sensor_token_dict[msg.header.frame_id]
            prev_data_token = data_token_dict[topic][0]
            data_token = data_token_dict[topic][1]
            data_token_dict[topic][1] = token_hex(16)
            data_token_dict[topic][0] = data_token
            sensor_data = dict()
            sensor_data['token'] = data_token
            sensor_data['sample_token'] = sample_token
            sensor_data['calibrated_sensor_token'] = sensor_token
            # Find closest ego pose
            previous_time_difference = np.inf
            for i in range(previous_loc, len(ego_pose_queue)):
                time_difference = abs((datetime.fromtimestamp(timestamp * 1e-9) - datetime.fromtimestamp(ego_pose_queue[i][1] * 1e-9)).total_seconds())
                if time_difference < previous_time_difference:
                    previous_time_difference = time_difference
                else:
                    ego_pose_token = ego_pose_queue[i-1]
                    ego_velocity = (ego_pose_queue[i][2] - ego_pose_queue[i-1][2]) / (datetime.fromtimestamp(ego_pose_queue[i][1] * 1e-9) - datetime.fromtimestamp(ego_pose_queue[i-1][1] * 1e-9)).total_seconds()
                    previous_loc = i - 1
                    break
            sensor_data['ego_pose_token'] = ego_pose_token[0]
            # Save data
            if topic in lidar_topics:
                sensor_name = lidar_topics[topic]
                height = 0
                width = 0
                saved_points = np.zeros((msg.width, 5))
                point_num = 0
                for point in read_points(msg, skip_nans=True, field_names=['x', 'y', 'z', 'intensity']):
                    saved_points[point_num,0]=point[0]
                    saved_points[point_num,1]=point[1]
                    saved_points[point_num,2]=point[2]
                    saved_points[point_num,3]=point[3]
                    point_num += 1
                if sensor_name not in sensors_added:
                    filename = "samples/{0}/{1}__{0}__{2}.pcd.bin".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    sensors_added.add(sensor_name)
                    is_key_frame = True
                    if first_sample == '':
                        first_sample = sample_token
                else:
                    filename = "sweeps/{0}/{1}__{0}__{2}.pcd.bin".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    is_key_frame = False
                with open(filename, 'wb') as pcd_file:
                    saved_points.astype('float32').tofile(pcd_file)
            elif topic in camera_topics:
                sensor_name = camera_topics[topic]
                img = cv2.cvtColor(img_converter.compressed_imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
                height = img.shape[0]
                width = img.shape[1]
                if sensor_name not in sensors_added:
                    filename = "samples/{0}/{1}__{0}__{2}.jpg".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    sensors_added.add(sensor_name)
                    is_key_frame = True
                    if first_sample == '':
                        first_sample = sample_token
                else:
                    filename = "sweeps/{0}/{1}__{0}__{2}.jpg".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    is_key_frame = False
                cv2.imwrite(filename, img)
            elif topic in radar_topics:
                sensor_name = radar_topics[topic]
                height = 0
                width = 0
                x_pos = msg.track_range*math.cos(math.radians(msg.track_angle))
                y_pos =  msg.track_range*math.sin(math.radians(msg.track_angle))
                vx_comp = msg.track_range_rate
                vy_comp = msg.track_lat_rate
                vx = vx_comp + ego_velocity[0]
                vy = vy_comp + ego_velocity[1]
                dyn_prop = not (vx < 1.0 and vy < 1.0)
                ambig_state = 3 if dyn_prop else 4
                points = np.array([x_pos,y_pos, 0.0, dyn_prop, msg.track_id, msg.track_width, vx, vy, vx_comp, vy_comp, 1, ambig_state, int(1/math.sqrt(2) * x_pos), int(1/math.sqrt(2) * y_pos), 0, 1, int(1/math.sqrt(2) * vx), int(1/math.sqrt(2) * vy)])
                if sensor_name not in sensors_added:
                    filename = "samples/{0}/{1}__{0}__{2}.pcd".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    sensors_added.add(sensor_name)
                    is_key_frame = True
                    if first_sample == '':
                        first_sample = sample_token
                else:
                    filename = "sweeps/{0}/{1}__{0}__{2}.pcd".format(sensor_name, rosbag_file.split('/')[-1], timestamp)
                    is_key_frame = False
                with open(filename, 'wb') as pcd_file:
                    pcd_file.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z dyn_prop id rcs vx vy vx_comp vy_comp is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms\nSIZE 4 4 4 1 2 4 4 4 4 4 1 1 1 1 1 1 1 1\nTYPE F F F I I F F F F F I I I I I I I I\nCOUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\nWIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA binary\n".encode('utf-8'))
                    points.tofile(pcd_file)
            sensor_data['filename'] = filename
            sensor_data['fileformat'] = 'pcd'
            sensor_data['is_key_frame'] = is_key_frame
            sensor_data['height'] = height
            sensor_data['width'] = width
            sensor_data['timestamp'] = timestamp * 1e-9
            sensor_data['prev'] = prev_data_token
            sensor_data['next'] = data_token_dict[topic][1]
            sample_data.append(sensor_data)         

    samples[-1]['next'] = ''
    sensors_cleared = set()
    for sensor_data in reversed(sample_data):
        if sensor_data['calibrated_sensor_token'] not in sensors_cleared:
            sensor_data['next'] = ''
            sensors_cleared.add(sensor_data['calibrated_sensor_token'])
        if len(sensors_cleared) == len(lidar_topics) + len(radar_topics) + len(camera_topics):
            break
    with open('v1.0-mini/sample.json', 'w') as f:
        json.dump(samples, f, indent=4)
    with open('v1.0-mini/sample_data.json', 'w') as f:
        json.dump(sample_data, f, indent=4)
    with open('v1.0-mini/ego_pose.json', 'w') as f:
        json.dump(ego_poses, f, indent=4)
            
    # Create scene.json
    if os.path.exists('v1.0-mini/scene.json'):
        with open('v1.0-mini/scene.json', 'r') as f:
            scenes = json.load(f)
    else:
        scenes = []
    with open('v1.0-mini/scene.json', 'w') as f:
        scene = dict()
        scene['token'] = scene_token
        scene['log_token'] = log_token
        scene['nbr_samples'] = nbr_samples
        scene['first_sample_token'] = first_sample
        scene['last_sample_token'] = sample_token
        if rosbag_file[-1] == '/':
            rosbag_file = rosbag_file[:-1]
        scene['name'] = rosbag_file.split('/')[-1]
        scene['description'] = param_dict["BAG_INFO"]["DESCRIPTION"]
        scenes.append(scene)
        json.dump(scenes, f, indent=4)

if __name__=="__main__":
    import argparse, argcomplete
    parser = argparse.ArgumentParser(description="Label point clouds with bounding boxes.")
    parser.add_argument("bag_in", type=str, help="Bag to load")
    parser.add_argument("param_file", type=str, help="Yaml file matching topics and tf frames to channel names")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    argdict : dict = vars(args)
    write_scene(argdict)
