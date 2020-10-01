import argparse

from ur_control import Robot
from transform3d import Transform

from iros_kit import layouts
from iros_kit.pose_extraction.extract_kit_t_objects import extract_kit_t_objects
from iros_kit.grasping.determine_grasps_and_order import determine_grasps_decreasing_clearance, load_grasp_config

from gripper import get_gripper


def from_dict(d, k):
    return d[k] if k in d else d['default']


parser = argparse.ArgumentParser()
parser.add_argument('--base-t-kitlayout', required=True)
parser.add_argument('--obj-names', nargs='+', default=None)
parser.add_argument('--layout', default=layouts.practice_fp)
parser.add_argument('--get-grasp-position', action='store_true')
parser.add_argument('--examine-grasp', action='store_true')
parser.add_argument('--stop-at-grasp', action='store_true')
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

base_t_kit = Transform.load(args.base_t_kitlayout)
kit_t_objects = extract_kit_t_objects(args.layout, debug=args.debug)
obj_grasp_configs, min_clearance = load_grasp_config(args.obj_names)
grasps, min_clearance = determine_grasps_decreasing_clearance(obj_grasp_configs, kit_t_objects,
                                                              min_clearance, debug=args.debug)

gripper = get_gripper()
gripper.open()

r = Robot.from_ip('192.168.1.130')
r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
          -1.0198443692973633, -4.752078358327047, -2.1280840078936976)
q_drop = (3.0432159900665283, -2.0951763592162074, -1.622988224029541,
          -0.9942649167827149, -4.733094040547506, -1.5899961630450647)

for name, tcp_t_obj, grasp_start_width, grasp_end_width in grasps:
    kit_t_obj = kit_t_objects[name]

    r.ctrl.moveJ(q_safe)

    grasp_start_width_mm = grasp_start_width * 1e3
    grasp_end_width_mm = grasp_end_width * 1e3

    if args.get_grasp_position:
        grasp_start_width_mm = 50
        grasp_end_width_mm = 0

    base_t_tcp_grasp = base_t_kit @ kit_t_obj @ tcp_t_obj.inv
    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))
    gripper.move(grasp_start_width_mm, 255, 255)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.grasp(grasp_end_width_mm, 0, 100, 50)
    if args.get_grasp_position:
        print(f'grasp pos of {name}: {gripper.get_position()} mm')
    if args.stop_at_grasp:
        quit()
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.1)))
    if args.examine_grasp:
        quit()
    r.ctrl.moveJ(q_drop)
    gripper.move(grasp_start_width_mm, 255, 255)
