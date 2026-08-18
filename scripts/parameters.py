"""NXP Jaguar Hardware Parameters and Motor Configuration for ROS 2.

RobStride RS00 Motors mapped across 2 CAN buses (can0 & can1).
"""

N_JOINTS = 12

# CAN Bus Interface assignment: can0 = Sisi Kanan (FR, BR), can1 = Sisi Kiri (FL, BL)
DEVICE = [
    "can1", "can1", "can1",  # BL (Back-Left)  -> can1
    "can0", "can0", "can0",  # BR (Back-Right) -> can0
    "can1", "can1", "can1",  # FL (Front-Left) -> can1
    "can0", "can0", "can0",  # FR (Front-Right)-> can0
]

# CAN Motor Node IDs (ID 1,2,3: Front; ID 4,5,6: Back)
CAN_ID = [
    4, 5, 6,  # BL: collar, hip, knee (can1)
    4, 5, 6,  # BR: collar, hip, knee (can0)
    1, 2, 3,  # FL: collar, hip, knee (can1)
    1, 2, 3,  # FR: collar, hip, knee (can0)
]

# Motor Model (NXP Jaguar uses RobStride RS00)
MOTOR_TYPE = [
    "RobStride00", "RobStride00", "RobStride00",
    "RobStride00", "RobStride00", "RobStride00",
    "RobStride00", "RobStride00", "RobStride00",
    "RobStride00", "RobStride00", "RobStride00",
]

# Motor Direction (+1 or -1)
MOTOR_DIR = [
     1, -1, -1,  # BL
     1,  1,  1,  # BR
    -1, -1, -1,  # FL
    -1,  1,  1,  # FR
]

MOTOR_OFFSET_THRE = [
    None, None, 4.0,
    None, None, None,
    None, None, 5.0,
    None, 5.0, None,
]

# Zero-calibration angular offsets (Adjust during bench calibration)
MOTOR_OFFSET_ANGLE = [
    -3.238870,  5.026955, -2.604017,
    -5.261047, -2.543738, -4.188643,
     4.011600,  2.632758, -1.578151,
     2.922098, -0.884712, -5.919000,
]

# ROS Hardware Joint Names (Order: BL, BR, FL, FR)
JOINT_NAME = [
    'BL_collar_joint', 'BL_hip_joint', 'BL_knee_joint',
    'BR_collar_joint', 'BR_hip_joint', 'BR_knee_joint',
    'FL_collar_joint', 'FL_hip_joint', 'FL_knee_joint',
    'FR_collar_joint', 'FR_hip_joint', 'FR_knee_joint',
]

# Standby Joint Angles (Folded/Sitting position)
STANDBY_ANGLE = [
     0.0,  0.8, -2.2,  # BL
     0.0,  0.8, -2.2,  # BR
     0.0,  0.8, -2.2,  # FL
     0.0,  0.8, -2.2,  # FR
]

# Nominal Standing Default Joint Angles in ROS Joint Order (BL, BR, FL, FR)
# Matches Isaac Lab q0: Roll: 0.0, Hip: -1.5, Knee: +1.5
DEFAULT_ANGLE = [
     0.0, -1.5,  1.5,  # BL
     0.0, -1.5,  1.5,  # BR
     0.0, -1.5,  1.5,  # FL
     0.0, -1.5,  1.5,  # FR
]

# PD Control Gains for RS00 Motors
KP_GAIN = [25.0] * 12
KD_GAIN = [1.0] * 12

# Isaac Lab RL Policy Control Parameters
ACTION_SCALE = 0.25
ACTION_CLIPPING = 10.0
CONTROL_HZ = 50       # 50 Hz control loop (dt = 0.02s)
CONTROL_DT = 0.02

CAN_HZ = 200          # 200 Hz CAN communication rate

# Remapping Indices: ROS CAN Order (BL, BR, FL, FR) <-> Isaac Lab Order (Rolls, Hips, Knees)
# Isaac Lab: [FR_r, FL_r, BR_r, BL_r, FR_h, FL_h, BR_h, BL_h, FR_k, FL_k, BR_k, BL_k]
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
