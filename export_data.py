#!/usr/bin/env python3

from nuscenes.nuscenes import NuScenes
import sys
import os
from tqdm import tqdm
import tarfile

if len(sys.argv) != 2:
    print("Usage: ./export_data.py {data path}")
    exit(1)

data_path = sys.argv[1]
nusc = NuScenes(version='v1.0-mini', dataroot=data_path, verbose=True)
current_scene = nusc.scene[0]["token"]
tar = tarfile.open("%s_%s.tar.gz" % (nusc.get('log', nusc.get('scene', current_scene)['log_token'])['vehicle'], nusc.get('scene', current_scene)['name'],), "w:gz")
for i in tqdm(range(len(nusc.sample_data))):
    sample_data = nusc.sample_data[i]
    scene_token = nusc.get('sample', sample_data['sample_token'])['scene_token']          
    if scene_token == current_scene:
        tar.add(os.path.join(data_path, sample_data["filename"]), arcname=sample_data["filename"])
    else:
        tar.close()
        current_scene = scene_token
        tar = tarfile.open("%s_%s.tar.gz" % (nusc.get('log', nusc.get('scene', current_scene)['log_token'])['vehicle'], nusc.get('scene', current_scene)['name'],), "w:gz")
tar.close()
tar = tarfile.open("metadata.tar.gz", "w:gz")
tar.add(os.path.join(data_path, "v1.0-mini"))
