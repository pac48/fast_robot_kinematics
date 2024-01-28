import numpy as np
from urdf_parser_py import urdf
from jinja2 import Template

with open("robot.urdf") as f:
    robot = urdf.Robot.from_xml_string(f.read())


def get_T(joint):
    rpy = joint.origin.rpy
    xyz = joint.origin.xyz
    T = np.eye(4)

    yaw = np.array([[np.cos(rpy[2]), np.sin(rpy[2]), 0],
                    [-np.sin(rpy[2]), np.cos(rpy[2]), 0],
                    [0, 0, 1]])

    pitch = np.array([[np.cos(rpy[1]), 0, np.sin(rpy[1])],
                      [0, 1, 0],
                      [-np.sin(rpy[1]), 0, np.cos(rpy[1])]])

    roll = np.array([[1, 0, 0],
                     [0, np.cos(rpy[0]), np.sin(rpy[0])],
                     [0, -np.sin(rpy[0]), np.cos(rpy[0])]])
    T[:3, :3] = yaw @ pitch @ roll
    T[:3, 3] = xyz

    return T


tip_link_name = "grasp_link"
root_link_name = "base_link"

rotations = []
offsets = []
types = []
joint_names = []
while tip_link_name != root_link_name:
    tip_joint_name, tip_link_name = robot.parent_map[tip_link_name]
    joint_names.append(tip_joint_name)

joint_names.reverse()

T_fixed = np.eye(4)
for joint_name in joint_names:
    joint = robot.joint_map[joint_name]
    if joint.type == 'fixed':
        T_fixed = T_fixed * get_T(joint)
    elif joint.type == 'revolute':
        T = get_T(joint)
        rotations.append(T[:3, :3])
        offsets.append(T[:3, 3])
        types.append('revolute')
    elif joint.type == 'prismatic':
        T = get_T(joint)
        rotations.append(T[:3, :3])
        offsets.append(T[:3, 3])
        types.append('prismatic')
    else:
        raise Exception(f"joint type {joint.type} in URDF not supported")

with open("robot_config.cpp.template", 'r') as f:
    j2_template = Template(f.read())
    code = j2_template.render({'rotations': rotations, 'offsets': offsets, 'types': types}, trim_blocks=True)

with open("forward_kinematics_test.cpp", 'w') as f:
    f.write(code)
