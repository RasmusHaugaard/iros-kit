import json
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import cv2
import trimesh
from transform3d import Transform

import svg_to_img

kit_size_m = 0.256


def get_signed_axis(text: str):
    if text.startswith('-'):
        sign = -1
        text = text[1:]
    else:
        sign = 1
    return np.eye(3)['xyz'.index(text)] * sign


def extract_kit_t_objects(svg_filepath, debug=False):
    img = svg_to_img.svg_to_numpy(svg_filepath)
    h, w = img.shape

    kit_t_objects = dict()

    objects = json.load(open('../objects.json'))
    objects = {name: obj for name, obj in objects.items() if 'up_kit' in obj}

    debug_img = np.tile(img[..., None], (1, 1, 3))

    for name, obj in objects.items():
        template = cv2.imread(f'../templates/{name}.png', cv2.IMREAD_GRAYSCALE)
        res = cv2.matchTemplate(img, template, cv2.TM_CCOEFF_NORMED)
        *_, (xmi, ymi) = cv2.minMaxLoc(res)
        xma, yma = xmi + template.shape[1], ymi + template.shape[0]
        cv2.rectangle(debug_img, (xmi, ymi), (xma, yma), (50, 50, 50), 1)
        cv2.putText(debug_img, name, (xmi, ymi), cv2.FONT_HERSHEY_PLAIN, 2, (50, 50, 50), 2)
        cx, cy = (xmi + xma) / 2, (ymi + yma) / 2

        # get rotation matrix
        up = get_signed_axis(obj['up_kit'])
        if 'forward_kit' in obj:
            forward = get_signed_axis(obj['forward_kit'])
        else:
            # choose a valid forward vector for objects with symmetry
            forward = np.eye(3)[np.argmin(np.abs(np.eye(3) @ up))]
        assert forward @ up == 0
        R = np.stack((
            np.cross(forward, up),
            forward,
            up
        ))
        assert np.linalg.det(R) == 1

        t = np.array((cx, h - cy, 0)) / h * kit_size_m + (0, 0, obj['height_offset_kit'])
        if 'center_offset_kit' in obj:
            t -= R @ obj['center_offset_kit']

        kit_t_objects[name] = Transform(R=R, p=t)

        if debug and 'step' in obj:
            stl_file = obj['step'].replace('.STEP', '.stl')
            verts_obj = trimesh.load_mesh(f'../../stl/{stl_file}').vertices * 1e-3
            verts_kit = verts_obj @ R.T + t
            z_min = verts_kit[:, 2].min()
            assert np.abs(z_min) < 1e-4

            # flip y for img frame, convert from m to px
            for axes, color in ([0, 1], (255, 0, 0)), ([0, 2], (0, 255, 0)):
                uv = (0, h - 1) + verts_kit[:, axes] * (1, -1) * h / kit_size_m
                uv = np.around(uv).astype(int)
                u, v = uv.T
                debug_img[v, u] = color

    if debug:
        plt.imshow(debug_img, interpolation='bilinear')
        plt.axis('off')
        plt.show()

    return kit_t_objects


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--layout', default='custom_0')
    args = parser.parse_args()

    pose_folder = Path(f'../kit_t_objects_{args.layout}')
    pose_folder.mkdir(exist_ok=True)

    kit_t_objects = extract_kit_t_objects(f'../layouts/{args.layout}.svg', debug=True)

    for name, kit_t_object in kit_t_objects.items():
        kit_t_object.save(pose_folder / f'kit_t_{name}')


if __name__ == '__main__':
    main()
