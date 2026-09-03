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
# Calibrated True Sitting Zero offsets (so that sitting position = 0.0 rad):
MOTOR_OFFSET_ANGLE = [
    +0.3245, -1.3483, -0.0988,  # BL (can1: Collar, Hip, Knee)
    -0.3517, -1.3476, -0.0539,  # BR (can0: Collar, Hip, Knee)
    +0.3526, -1.2127, -0.0967,  # FL (can1: Collar, Hip, Knee)
    -0.1881, -1.1767, -0.2427,  # FR (can0: Collar, Hip, Knee)
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

# Relax Pose Joint Angles (Equal to calibrated motor offsets; raw motor reading = 0.0 rad)
RELAX_ANGLE = MOTOR_OFFSET_ANGLE.copy()

# Nominal Standing Default Joint Angles in ROS Joint Order (BL, BR, FL, FR)
# Matches Isaac Lab q0:
#   BL & BR (Back Legs):  Roll: 0.0, Hip: -1.55, Knee: +1.35
#   FL & FR (Front Legs): Roll: 0.0, Hip: -1.55, Knee: +1.35
DEFAULT_ANGLE = [
     0.0, -1.55,  1.35,  # BL
     0.0, -1.55,  1.35,  # BR
     0.0, -1.55,  1.35,  # FL
     0.0, -1.55,  1.35,  # FR
]

import os
import yaml

# Dynamic parameter loader from config/sim2real.yaml (Single Source of Truth)
_CONFIG_FILE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "sim2real.yaml"))

_coxa_kp = 20.0
_coxa_kd = 1.5
_pitch_kp = 25.0
_pitch_kd = 1.5
_can_hz = 200

if os.path.isfile(_CONFIG_FILE):
    try:
        with open(_CONFIG_FILE, "r") as _f:
            _cfg = yaml.safe_load(_f)
        if _cfg and "robstride_can_hardware" in _cfg:
            _hw = _cfg["robstride_can_hardware"].get("ros__parameters", {})
            _coxa_kp = float(_hw.get("default_coxa_kp", _coxa_kp))
            _coxa_kd = float(_hw.get("default_coxa_kd", _coxa_kd))
            _pitch_kp = float(_hw.get("default_kp", _pitch_kp))
            _pitch_kd = float(_hw.get("default_kd", _pitch_kd))
            _can_hz = int(_hw.get("rate_hz", _can_hz))
    except Exception:
        pass

# PD Control Gains for RS00 Motors (ROS Order: BL, BR, FL, FR)
KP_GAIN = [
    _coxa_kp, _pitch_kp, _pitch_kp,  # BL: collar, hip, knee
    _coxa_kp, _pitch_kp, _pitch_kp,  # BR: collar, hip, knee
    _coxa_kp, _pitch_kp, _pitch_kp,  # FL: collar, hip, knee
    _coxa_kp, _pitch_kp, _pitch_kp,  # FR: collar, hip, knee
]
KD_GAIN = [
    _coxa_kd, _pitch_kd, _pitch_kd,  # BL: collar, hip, knee
    _coxa_kd, _pitch_kd, _pitch_kd,  # BR: collar, hip, knee
    _coxa_kd, _pitch_kd, _pitch_kd,  # FL: collar, hip, knee
    _coxa_kd, _pitch_kd, _pitch_kd,  # FR: collar, hip, knee
]

# Isaac Lab RL Policy Control Parameters
ACTION_SCALE = 0.25
ACTION_CLIPPING = 10.0
CONTROL_HZ = 50       # 50 Hz control loop (dt = 0.02s)
CONTROL_DT = 0.02

CAN_HZ = _can_hz          # CAN communication rate loaded from YAML

# Remapping Indices: ROS CAN Order (BL, BR, FL, FR) <-> Isaac Lab Order (Rolls, Hips, Knees)
# Isaac Lab: [FR_r, FL_r, BR_r, BL_r, FR_h, FL_h, BR_h, BL_h, FR_k, FL_k, BR_k, BL_k]
ROS_TO_ISAAC = [9, 6, 3, 0, 10, 7, 4, 1, 11, 8, 5, 2]
ISAAC_TO_ROS = [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8]
