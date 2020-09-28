import json

import numpy as np
import trimesh

objects = json.load(open('../objects.json'))
objects = {key: val for key, val in objects.items() if 'up_kit' in val}


def get_signed_axis(text: str):
    if text.startswith('-'):
        sign = -1
        text = text[1:]
    else:
        sign = 1
    return np.eye(3)['xyz'.index(text)] * sign


for name, obj in objects.items():
    if name == 'belt':
        continue
    up = get_signed_axis(obj['up_kit'])
    stl_file = obj['step'].replace('.STEP', '.stl')
    verts = trimesh.load_mesh(f'../stl/{stl_file}').vertices * 1e-3

    center_height = -np.min(verts @ up)
    print(name, center_height)
