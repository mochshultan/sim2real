"""NXP Jaguar Hardware Parameters and Motor Configuration for ROS 2.

RobStride RS00 Motors mapped across 2 CAN buses (can0 & can1).
"""

from project_paths import load_default_config
from joint_layout import ROS_JOINT_NAMES, ROS_TO_ISAAC


N_JOINTS = 12
RS00_TORQUE_LIMIT_NM = 14.0
"""RobStride RS00 firmware torque limit [N·m]."""

RS00_CONTROL_TORQUE_LIMIT_NM = RS00_TORQUE_LIMIT_NM
"""Normal-control torque ceiling matching the RS00 hardware maximum [N·m]."""

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
    +0.3245, -1.2683, -0.1988,  # BL (can1: Collar, Hip, Knee)
    -0.3017, -1.2676, -0.1339,  # BR (can0: Collar, Hip, Knee)
    +0.1326, -1.1627, -0.3467,  # FL (can1: Collar, Hip, Knee)
    -0.1881, -1.1667, -0.2227,  # FR (can0: Collar, Hip, Knee)
]

# ROS Hardware Joint Names (Order: BL, BR, FL, FR)
JOINT_NAME = ROS_JOINT_NAMES

# Standby Joint Angles (Folded/Sitting position = 0.0 rad after mechanical zeroing)
STANDBY_ANGLE = [
     0.0,  0.0,  0.0,  # BL
     0.0,  0.0,  0.0,  # BR
     0.0,  0.0,  0.0,  # FL
     0.0,  0.0,  0.0,  # FR
]


# Tuned standing joint angles [rad] in ROS order (BL, BR, FL, FR).
# Separate from each checkpoint's observation/action reference.
STAND_ANGLE = [
     0.0, -1.59,  1.38,  # BL: coxa, hip, knee
     0.0, -1.59,  1.38,  # BR: coxa, hip, knee
     0.0, -1.59,  1.40,  # FL: coxa, hip, knee
     0.0, -1.59,  1.40,  # FR: coxa, hip, knee
]

# Trained policy reference joint angles in ROS joint order (BL, BR, FL, FR).
# Matches the deployed Jaguar AdaBoot Ideal L2C2 checkpoint 29996.
DEFAULT_ANGLE = [
     0.0, -1.49,  1.20,  # BL
     0.0, -1.49,  1.20,  # BR
     0.0, -1.51,  1.22,  # FL
     0.0, -1.51,  1.22,  # FR
]

# Shared defaults for standalone hardware/calibration tools. ROS nodes can
# override these defaults through their parameter files at launch.
_hw = load_default_config()["robstride_can_hardware"]["ros__parameters"]
_coxa_kp = float(_hw["default_coxa_kp"])
_coxa_kd = float(_hw["default_coxa_kd"])
_pitch_kp = float(_hw["default_kp"])
_pitch_kd = float(_hw["default_kd"])
_knee_kp = float(_hw["default_knee_kp"])
_knee_kd = float(_hw["default_knee_kd"])
_can_hz = int(_hw["rate_hz"])

# PD Control Gains for RS00 Motors (ROS Order: BL, BR, FL, FR)
KP_GAIN = [
    _coxa_kp, _pitch_kp, _knee_kp,  # BL: collar, hip, knee
    _coxa_kp, _pitch_kp, _knee_kp,  # BR: collar, hip, knee
    _coxa_kp, _pitch_kp, _knee_kp,  # FL: collar, hip, knee
    _coxa_kp, _pitch_kp, _knee_kp,  # FR: collar, hip, knee
]
KD_GAIN = [
    _coxa_kd, _pitch_kd, _knee_kd,  # BL: collar, hip, knee
    _coxa_kd, _pitch_kd, _knee_kd,  # BR: collar, hip, knee
    _coxa_kd, _pitch_kd, _knee_kd,  # FL: collar, hip, knee
    _coxa_kd, _pitch_kd, _knee_kd,  # FR: collar, hip, knee
]

CAN_HZ = _can_hz
