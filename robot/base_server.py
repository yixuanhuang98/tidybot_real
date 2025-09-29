# Author: Jimmy Wu
# Date: October 2024
#
# This RPC server allows other processes to communicate with the mobile base
# low-level controller, which runs in its own, dedicated real-time process.
#
# Note: Operations that are not time-sensitive should be run in a separate,
# non-real-time process to avoid interfering with the low-level control and
# causing latency spikes.

import time
import numpy as np
from multiprocessing.managers import BaseManager as MPBaseManager
from base_controller import Vehicle
from constants import BASE_RPC_HOST, BASE_RPC_PORT, RPC_AUTHKEY
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface

class Base:
    def __init__(self, max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79)):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.vehicle = None

    def reset(self):
        # Stop low-level control
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()

        # Create new instance of vehicle
        self.vehicle = Vehicle(max_vel=self.max_vel, max_accel=self.max_accel)
        print('vehicle', self.vehicle)

        # Start low-level control
        self.vehicle.start_control()
        while not self.vehicle.control_loop_running:
            time.sleep(0.01)

    def execute_action(self, action):
        # self.vehicle.set_target_position(action['base_pose'])
        self.vehicle.set_target_position(np.array([-action['base_pose'][0], -action['base_pose'][1], action['base_pose'][2]]))

    def get_state(self):
        state = {'base_pose': self.vehicle.x}
        return state

    def get_pose(self):
        # return self.vehicle.x
        return -self.vehicle.x[0], -self.vehicle.x[1], self.vehicle.x[2]

    # def get_goal_reached(self, action):
    #     print('goal check', self.vehicle.otg_res == Result.Finished)
    #     return self.vehicle.otg_res == Result.Finished
    
    def get_goal_reached(self, action, tol=0.01):
        # Check if robot is within tolerance of the target
        target_pose = action
        current_pose = self.get_pose()
        
        # Compute position distance
        position_dist = np.linalg.norm(target_pose[:2] - current_pose[:2])
        
        # Compute heading difference  
        heading_diff = abs(target_pose[2] - current_pose[2])
        heading_diff = min(heading_diff, 2*np.pi - heading_diff)  # Handle wrap-around
        
        # Goal reached if within position tolerance (2cm) and heading tolerance (5 degrees)
        return position_dist < tol and heading_diff < np.radians(5)
    
    # def get_goal_reached(self, action, tol=0.005):
    #     # print('action:', action)
    #     # print('pose:', self.vehicle.x)
        
    #     # Compute Euclidean distance between action and current pose
    #     dist = np.linalg.norm(action - self.vehicle.x)
    #     # print('distance:', dist)

    #     return dist < tol

    # def get_goal_reached(self, action):
    #     print('action', action)
    #     print('pose', self.vehicle.x)
    #     if np.close(action, self.vehicle.x) < 0.01:
    #         return True
    #     else:
    #         return False

    def close(self):
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()

class BaseManager(MPBaseManager):
    pass

BaseManager.register('Base', Base)

if __name__ == '__main__':
    manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    print(f'Base manager server started at {BASE_RPC_HOST}:{BASE_RPC_PORT}')
    server.serve_forever()
    # print('enter')
    # import numpy as np
    # from constants import POLICY_CONTROL_PERIOD
    # manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    # manager.connect()
    # base = manager.Base()
    # try:
    #     base.reset()
    #     for i in range(50):
    #         base.execute_action({'base_pose': np.array([(i / 50) * 0.5, 0.0, 0.0])})
    #         print(base.get_state())
    #         time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    # finally:
    #     base.close()
