from jinja2 import Template
from urdf_parser_py import urdf

import argparse
import numpy as np


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file')
    parser.add_argument('fk_template')
    parser.add_argument('fk_output_file')
    parser.add_argument('root_link_name')
    parser.add_argument('tip_link_name')
    args = parser.parse_args()

    root_link_name = args.root_link_name
    tip_link_name = args.tip_link_name
    with open(args.urdf_file) as f:
        robot = urdf.Robot.from_xml_string(f.read())

    def get_transform(joint):
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
            T_fixed = T_fixed * get_transform(joint)
        elif joint.type == 'revolute':
            # TODO: need to add joint limits?
            T = get_transform(joint) @ T_fixed
            T_fixed = np.eye(4)
            rotations.append(T[:3, :3])
            offsets.append(T[:3, 3])
            types.append('revolute')
        elif joint.type == 'continuous':
            T = get_transform(joint) @ T_fixed
            T_fixed = np.eye(4)
            rotations.append(T[:3, :3])
            offsets.append(T[:3, 3])
            types.append('revolute')
        elif joint.type == 'prismatic':
            # TODO: need to add joint limits?
            T = get_transform(joint) @ T_fixed
            T_fixed = np.eye(4)
            rotations.append(T[:3, :3])
            offsets.append(T[:3, 3])
            types.append('prismatic')
        else:
            raise Exception(f"joint type {joint.type} in URDF not supported")

    # truncate near zero values
    offsets = [val * (np.abs(val) > 1E-5) for val in offsets]
    rotations = [val * (np.abs(val) > 1E-5) for val in rotations]

    with open(args.fk_template, 'r') as f:
        j2_template = Template(f.read())
        code = j2_template.render({'rotations': rotations, 'offsets': offsets, 'types': types}, trim_blocks=True)

    with open(args.fk_output_file, 'w') as f:
        f.write(code)

    print(f"FAST_FK_NUMBER_OF_JOINTS={len(types)}", end="")


if __name__ == "__main__":
    run()
