import numpy as np
from constants import POLICY_CONTROL_PERIOD
import time

from constants import BASE_RPC_HOST, BASE_RPC_PORT, ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from constants import BASE_CAMERA_SERIAL
from base_server import BaseManager

manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
base = manager.Base()
try:
    base.reset()
    for i in range(50):
        # base.execute_action({'base_pose': np.array([(i / 50) * 0.5, 0.0, 0.0])})
        base.execute_action({'base_pose': np.array([0.2, 0.0, 0.0])})
        print(base.get_state())
        time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
finally:
    base.close()