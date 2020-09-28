import argparse
from ur_control import Robot
from transform3d import Transform
import numpy as np
from gripper import get_gripper

parser = argparse.ArgumentParser()
parser.add_argument('obj_name')
args = parser.parse_args()

obj_name = args.obj_name

gripper = get_gripper()
gripper.open()

r = Robot.from_ip('192.168.1.130')
q_safe = (3.317805290222168, -1.3769071859172364, -1.9077906608581543,
          -1.4512933057597657, -4.752126518880026, -1.590332333241598)
r.ctrl.moveJ(q_safe)

base_t_kit = Transform.load('base_t_kitlayout')
kit_t_obj = Transform.load(f'../pose_extraction/kit_t_objects_practice/kit_t_{obj_name}')

base_t_tcp = base_t_kit @ Transform(p=(*kit_t_obj.p[:2], 0.25), rotvec=(np.pi, 0, 0))
r.ctrl.moveL(base_t_tcp)

r.ctrl.teachMode()
input('enter when grasp pose is good')
r.ctrl.endTeachMode()

base_t_tcp = r.base_t_tcp()
tcp_t_obj = base_t_tcp.inv @ base_t_kit @ kit_t_obj

print('tcp_t_obj:')
for val in tcp_t_obj:
    print(val)
