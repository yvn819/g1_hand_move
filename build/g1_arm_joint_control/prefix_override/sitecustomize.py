import sys
if sys.prefix == '/home/yutong/miniconda3/envs/g1_control':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yutong/scaledrive.ai/fincantieri/g1_hand_move/install/g1_arm_joint_control'
