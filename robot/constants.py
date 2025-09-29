import numpy as np

################################################################################
# Mobile base

# Vehicle center to steer axis (m)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka
# h_x, h_y = 0.140150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.120150 * np.array([-1.0, 1.0, 1.0, -1.0])  # ARX5

# Encoder magnet offsets
ENCODER_MAGNET_OFFSETS = [1988.0 / 4096, 491.0 / 4096, 1266.0 / 4096, 822.0 / 4096]
# ENCODER_MAGNET_OFFSETS = [0.0 / 4096, 0.0 / 4096, 0.0 / 4096, 0.0 / 4096]  # TODO

################################################################################
# Teleop and imitation learning

# Base and arm RPC servers
BASE_RPC_HOST = 'localhost'
BASE_RPC_PORT = 50000
ARM_RPC_HOST = 'localhost'
ARM_RPC_PORT = 50001
RPC_AUTHKEY = b'secret password'

# Cameras
BASE_CAMERA_SERIAL = '7DEAE8DE'
# WRIST_CAMERA_SERIAL = 'TODO'  # Not used by Kinova wrist camera

# Policy
POLICY_SERVER_HOST = 'localhost'
POLICY_SERVER_PORT = 5555
POLICY_CONTROL_FREQ = 10
POLICY_CONTROL_PERIOD = 1.0 / POLICY_CONTROL_FREQ
POLICY_IMAGE_WIDTH = 84
POLICY_IMAGE_HEIGHT = 84

## old one

SERVER_HOSTNAME = '192.168.0.11' # 'bohg-ws-14'
ROBOT_HOSTNAME_PREFIX = '192.168.0.60'
CONN_AUTHKEY = b'secret password'
REDIS_PASSWORD = 'secret password'

################################################################################
# Arm

MOUNTING_OFFSET = 0.12 # for new kinova mounting offset
HEIGHT_OFFSET = -0.288 - 0.06 # maximum: -0.288 - 0.077, for new kinova height offset

# Arm-dependent heading compensation (set to 0 if unsure)
ARM_HEADING_COMPENSATION = {
    0: -0.7,  # Robot 1 (asset tag: none)
    1: 0.2,   # Robot 2 (asset tag: 000007 402760)
    2: 0.7,   # Robot 3 (asset tag: 000007 402746)
}

################################################################################
# Camera

CAMERA_SERIALS = {
    0: '7DEAE8DE',  # Robot 1
    1: '44251E9E',  # Robot 2
    2: '7E841E9E',  # Robot 3
}
CAMERA_FOCUS = 0
CAMERA_TEMPERATURE = 3900
CAMERA_EXPOSURE = 156
CAMERA_GAIN = 10
