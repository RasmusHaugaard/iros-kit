import json

import numpy as np
import trimesh

from iros_kit.pose_extraction.extract_kit_t_objects import get_signed_axis

obj_config = json.load(open('../obj_config.json'))
cad_files = json.load(open('../../cad_files.json'))

for name, config in obj_config.items():
    up = get_signed_axis(config['up_kit'])
    stl_file = f'../../stl/{cad_files[name]}'
    verts = trimesh.load_mesh().vertices * 1e-3
    height_offset = -np.min(verts @ up)
    print(name, height_offset)
