import time
import argparse
import json

from ur_control import Robot
from transform3d import Transform

from gripper import get_gripper
from determine_grasp_and_order import determine_grasps_robust, load_objects


def from_dict(d, k):
    return d[k] if k in d else d['default']


parser = argparse.ArgumentParser()
parser.add_argument('--obj-names', nargs='+', default=None)
parser.add_argument('--layout', default='practice')
parser.add_argument('--wait', type=float, default=1.)
parser.add_argument('--dont-put-back', action='store_true')
parser.add_argument('--stop-at-grasp', action='store_true')
args = parser.parse_args()

grasp_config = json.load(open('grasp_config.json'))
objects = load_objects(grasp_config, 'custom_0')

gripper = get_gripper()
gripper.open()

r = Robot.from_ip('192.168.1.130')
r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
          -1.0198443692973633, -4.752078358327047, -2.1280840078936976)

grasps = determine_grasps_robust(objects, grasp_config['grasp_other_clearance'])
base_t_kit = Transform.load('base_t_kitlayout')

for name, kit_t_tcp, grasp_width in grasps:
    r.ctrl.moveJ(q_safe)

    base_t_tcp_grasp = base_t_kit @ kit_t_tcp
    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))
    gripper.move(grasp_width * 1e3, 255, 255)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.grasp(0, 0, 100, 0)
    if args.stop_at_grasp:
        quit()
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
    if args.dont_put_back:
        quit()
    time.sleep(args.wait)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.move(grasp_width * 1e3, 255, 255)
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
