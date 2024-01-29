# fast_robot_kinematics

Compile a URDF into an optimized C++ library for fast forward kinematics.

| robot | kinematic chain                       | FK time (ns) |
|-------|---------------------------------------|--------------|
| PR2   | base_link l_gripper_r_finger_tip_link | 60           |
| UR5e  | base_link -> grasp_link               | 40           |
