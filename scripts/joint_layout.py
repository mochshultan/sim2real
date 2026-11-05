"""Joint names and permutations shared by controller, drivers and diagnostics."""

ISAAC_JOINT_NAMES = [
    "Fr_roll_joint", "Fl_roll_joint", "Br_roll_joint", "Bl_roll_joint",
    "Fr_hip_pitch_joint", "Fl_hip_pitch_joint", "Br_hip_pitch_joint", "Bl_hip_pitch_joint",
    "Fr_knee_joint", "Fl_knee_joint", "Br_knee_joint", "Bl_knee_joint",
]
ROS_JOINT_NAMES = [
    f"{leg}_{joint}_joint"
    for leg in ("BL", "BR", "FL", "FR")
    for joint in ("collar", "hip", "knee")
]
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ROS_NAME_TO_ISAAC_IDX = {
    name: isaac_index
    for isaac_index, ros_index in enumerate(ROS_TO_ISAAC)
    for name in (ISAAC_JOINT_NAMES[isaac_index], ROS_JOINT_NAMES[ros_index])
}
NAME_TO_ROS_INDEX = {
    name: ROS_TO_ISAAC[isaac_index]
    for name, isaac_index in ROS_NAME_TO_ISAAC_IDX.items()
}
