ARM_JOINT_NAMES = [
    'L_SHOULDER_PITCH',
    'L_SHOULDER_ROLL',
    'L_SHOULDER_YAW',
    'L_ELBOW',
    'L_WRIST_ROLL',
    'L_WRIST_PITCH',
    'L_WRIST_YAW',
    'R_SHOULDER_PITCH',
    'R_SHOULDER_ROLL',
    'R_SHOULDER_YAW',
    'R_ELBOW',
    'R_WRIST_ROLL',
    'R_WRIST_PITCH',
    'R_WRIST_YAW',
]

# Matches Unitree G1 29-DOF LowCmd index order for arms.
ARM_JOINT_INDICES = [
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
]

ARM_DOF = len(ARM_JOINT_INDICES)
