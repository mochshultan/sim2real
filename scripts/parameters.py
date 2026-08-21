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

# Zero-calibration angular offsets
# Offsets calibrated from actual manual sit pose so that sitting position = 0.0 rad:
# BL: collar=+0.3756, hip=-1.3275, knee=-0.1064
# BR: collar=-0.3894, hip=-1.3267, knee=-0.0259
# FL: collar=+0.3407, hip=-1.2098, knee=+0.0013
# FR: collar=-0.2571, hip=-1.1695, knee=-0.1402
MOTOR_OFFSET_ANGLE = [
     0.3756, -1.3275, -0.1064,  # BL (can1)
    -0.3894, -1.3267, -0.0259,  # BR (can0)
      0.3407, -1.2098,  0.0013,  # FL (can1)
    -0.2571, -1.1695, -0.1402,  # FR (can0)
]

# ROS Hardware Joint Names (Order: BL, BR, FL, FR)
JOINT_NAME = [
    'BL_collar_joint', 'BL_hip_joint', 'BL_knee_joint',
    'BR_collar_joint', 'BR_hip_joint', 'BR_knee_joint',
    'FL_collar_joint', 'FL_hip_joint', 'FL_knee_joint',
    'FR_collar_joint', 'FR_hip_joint', 'FR_knee_joint',
]

# Standby Joint Angles (Folded/Sitting position = 0.0 rad after mechanical zeroing)
STANDBY_ANGLE = [
     0.0,  0.0,  0.0,  # BL
     0.0,  0.0,  0.0,  # BR
     0.0,  0.0,  0.0,  # FL
     0.0,  0.0,  0.0,  # FR
]

# Nominal Standing Default Joint Angles in ROS Joint Order (BL, BR, FL, FR)
# Matches Isaac Lab q0:
#   BL & BR (Back Legs):  Roll: 0.0, Hip: -1.40, Knee: +1.36 (Tuned)
#   FL & FR (Front Legs): Roll: 0.0, Hip: -1.50, Knee: +1.40
DEFAULT_ANGLE = [
     0.0, -1.40,  1.36,  # BL
     0.0, -1.40,  1.36,  # BR
     0.0, -1.50,  1.40,  # FL
     0.0, -1.50,  1.40,  # FR
]

# PD Control Gains for RS00 Motors
KP_GAIN = [25.0] * 12
KD_GAIN = [1.5] * 12

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
