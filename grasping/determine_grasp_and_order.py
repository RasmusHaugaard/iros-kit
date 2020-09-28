import json
from pathlib import Path

import numpy as np
import trimesh
from trimesh.creation import extrude_polygon
from trimesh.collision import CollisionManager  # requires libfcl-dev and python-fcl
from shapely.geometry import Polygon
from transform3d import Transform


def from_dict(d, k):
    return d[k] if k in d else d['default']


def rotational_grasp_symmetry(kit_t_tcp_grasp: Transform, deg: int, offset=(0, 0, 0)):
    assert 360 % deg == 0
    n = 360 // deg
    to = Transform(p=offset)
    return [kit_t_tcp_grasp @ to.inv @ Transform(rotvec=(0, 0, np.deg2rad(i * deg))) @ to for i in range(n)]


def build_finger_grasp_volume(grasp_width: float, w=0.032, h=0.035,
                              bottom_width=0.007, base_height=0.005,
                              tcp_z_finger_bottom=0.1915):
    one_side = [(bottom_width, 0), (w, h - base_height), (w, h * 3)]
    one_side = np.array(one_side) + (grasp_width, 0)
    poly = np.concatenate((one_side, one_side[::-1] * (-1, 1)))
    poly = Polygon(poly)
    mesh = extrude_polygon(poly, w)
    mesh.apply_translation((0, 0, -w / 2))
    mesh.apply_transform(Transform(rotvec=(-np.pi / 2, 0, 0)).matrix)
    mesh.apply_translation((0, 0, tcp_z_finger_bottom))
    return mesh


def determine_grasps(
        objects,  # List[(name, mesh, kit_t_o, tcp_t_o, grasp_width, sym_deg, sym_offset)]
        min_clearance, debug=False, debug_view_res=(800, 500)
):
    if debug:
        scene = trimesh.Scene()
        for name, mesh, kit_t_o, *_ in objects:
            scene.add_geometry(mesh, transform=kit_t_o.matrix, geom_name=name)
        scene.show(resolution=debug_view_res)

    lense = trimesh.load_mesh('../stl/lense.stl')
    lense.apply_scale(1e-3)
    col_tool = CollisionManager()
    col_tool.add_object('lense', lense)

    col_objects = CollisionManager()
    for name, mesh, kit_t_o, *_ in objects:
        col_objects.add_object(name, mesh, transform=kit_t_o.matrix)

    worst_possible_clearance = np.inf
    grasps = []
    candidates = [*objects]
    clearances = []
    i = 0
    while candidates:
        candidate = candidates[i]
        name, mesh, kit_t_o, tcp_t_o, grasp_width, sym_deg, sym_offset = candidate

        # move away the grasp obj to ignore obj-tcp collision
        col_objects.set_transform(name, Transform(p=(1000, 0, 0)).matrix)

        finger_grasp_volume = build_finger_grasp_volume(grasp_width)
        col_tool.add_object('finger_grasp_volume', finger_grasp_volume)

        kit_t_tcp = kit_t_o @ tcp_t_o.inv
        ts = rotational_grasp_symmetry(kit_t_tcp, sym_deg, sym_offset)
        dists, dists_name = [], []
        for t in ts:
            col_tool.set_transform('lense', t.matrix)
            col_tool.set_transform('finger_grasp_volume', t.matrix)
            dist, dist_names = col_objects.min_distance_other(col_tool, return_names=True)
            if dist < worst_possible_clearance:
                worst_possible_clearance = dist
            dists.append(dist)
            dists_name.append(dist_names[0])
        di = np.argmax(dists).item()
        kit_t_tcp, dist, dist_name = ts[di], dists[di], dists_name[di]

        col_tool.remove_object('finger_grasp_volume')

        if dist > min_clearance:
            grasps.append((name, kit_t_tcp, grasp_width))
            candidates.remove(candidate)
            clearances.append(dist)
            i = 0
            print(f'{name}: {dist:.3f}')

            if debug:
                scene.add_geometry(lense, transform=kit_t_tcp.matrix, geom_name='lense')
                scene.add_geometry(finger_grasp_volume, transform=kit_t_tcp.matrix, geom_name='finger_grasp_volume')
                scene.show(resolution=debug_view_res)
                scene.delete_geometry([name, 'lense', 'finger_grasp_volume'])
        else:
            print(f'!! {name}: {dist_name} {dist:.3f} - changing order')
            # move the grasp object back in place
            col_objects.set_transform(name, kit_t_o.matrix)
            i += 1
            if i == len(candidates):
                raise RuntimeError('cant find solution with this clearance')

    return grasps


def determine_grasps_robust(objects, min_clearance, debug=False, debug_view_res=(800, 500),
                            attempts=10):
    i = 0
    while True:
        try:
            grasps = determine_grasps(objects, min_clearance)
            break
        except RuntimeError as e:
            print()
            i += 1
            if i == attempts:
                raise e
            min_clearance *= 0.75
    if debug:
        determine_grasps(objects, min_clearance, debug=True, debug_view_res=debug_view_res)
    return grasps, min_clearance


def load_objects(grasp_config: dict, layout: str):
    kit_root = Path(__file__).parent.parent

    grasp_order_desired = grasp_config['grasp_order_desired']
    cad_files = json.load(open(kit_root / 'cad_files.json'))

    objects = []
    for name in grasp_order_desired:
        grasp_width = grasp_config['grasp_width'][name]
        grasp_width += from_dict(grasp_config['grasp_width_clearance'], name)
        sym_deg = grasp_config['grasp_sym_rot_deg'][name]
        sym_offset = from_dict(grasp_config['grasp_sym_offset'], name)

        mesh = trimesh.load_mesh(kit_root / 'stl' / f'{cad_files[name]}.stl')
        mesh.apply_scale(1e-3)
        kit_t_o = Transform.load(kit_root / 'pose_extraction' / f'kit_t_objects_{layout}' / f'kit_t_{name}')
        tcp_t_o = Transform.load(kit_root / 'grasping' / 'tcp_t_obj_grasp' / f'tcp_t_{name}_grasp')
        objects.append((name, mesh, kit_t_o, tcp_t_o, grasp_width, sym_deg, sym_offset))

    return objects


def main():
    grasp_config = json.load(open('grasp_config.json'))
    layout = 'custom_0'
    objects = load_objects(grasp_config, layout)
    grasps, min_clearance = determine_grasps_robust(objects, grasp_config['grasp_other_clearance'], debug=True)
    print([g[0] for g in grasps])
    print(min_clearance)


if __name__ == '__main__':
    main()
