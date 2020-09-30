import time
import argparse

from ur_control import Robot
from transform3d import Transform

from iros_kit.layouts import practice_fp, custom_0_fp
from iros_kit.pose_extraction.extract_kit_t_objects import extract_kit_t_objects
from iros_kit.grasping.determine_grasps_and_order import determine_grasps_robust, load_grasp_config

from gripper import get_gripper


def from_dict(d, k):
    return d[k] if k in d else d['default']


parser = argparse.ArgumentParser()
parser.add_argument('--obj-names', nargs='+', default=None)
parser.add_argument('--custom_0', default='store_true')
parser.add_argument('--wait', type=float, default=1.)
parser.add_argument('--dont-put-back', action='store_true')
parser.add_argument('--stop-at-grasp', action='store_true')
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

gripper = get_gripper()
gripper.open()

r = Robot.from_ip('192.168.1.130')
r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
          -1.0198443692973633, -4.752078358327047, -2.1280840078936976)
q_drop =  # TODO

base_t_kit = Transform.load('base_t_kitlayout')
kit_t_objects = extract_kit_t_objects(custom_0_fp if args.custom_0 else practice_fp)
obj_grasp_configs, min_clearance = load_grasp_config(args.obj_names)
grasps, min_clearance = determine_grasps_robust(obj_grasp_configs, kit_t_objects, min_clearance, debug=args.debug)

for name, tcp_t_obj, grasp_start_width, grasp_end_width in grasps:
    kit_t_obj = kit_t_objects[name]

    r.ctrl.moveJ(q_safe)

    base_t_tcp_grasp = base_t_kit @ kit_t_obj @ tcp_t_obj.inv
    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))
    gripper.move(grasp_start_width * 1e3, 255, 255)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.grasp(grasp_end_width, 0, 100, 0)
    if args.stop_at_grasp:
        quit()
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
    if args.dont_put_back:
        quit()
    time.sleep(args.wait)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.move(grasp_start_width, 255, 255)
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
