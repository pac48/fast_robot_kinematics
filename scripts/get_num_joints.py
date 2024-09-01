from urdf_parser_py import urdf
import argparse


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file')
    parser.add_argument('root_link_name')
    parser.add_argument('tip_link_name')
    args = parser.parse_args()

    root_link_name = args.root_link_name
    tip_link_name = args.tip_link_name
    with open(args.urdf_file) as f:
        robot = urdf.Robot.from_xml_string(f.read())

    joint_names = []
    while tip_link_name != root_link_name:
        tip_joint_name, tip_link_name = robot.parent_map[tip_link_name]
        joint_names.append(tip_joint_name)

    print(f"FAST_FK_NUMBER_OF_JOINTS={len(joint_names)}", end="")


if __name__ == "__main__":
    run()
