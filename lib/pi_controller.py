import numpy as np
import RobotDART as rd

class PIController:
    def __init__(self, target, dt, Kp = 10., Ki = 0.1):
        self._target = target
        self._dt = dt
        self._Kp = Kp
        self._Ki = Ki
        self._sum_error = 0
    
    # Continuous updates on target's position
    def set_target(self, target):
        self._target = target
    
    # For resetting controller
    def reset_sum_error(self):
        self._sum_error = 0
        print(self._sum_error)

    # Function to compute error
    def error(self, tf):
        # Calculate rotation error
        rot_error = rd.math.logMap(self._target.rotation() @ tf.rotation().T)
        # Calculate translation error
        lin_error = self._target.translation() - tf.translation()
        return np.r_[rot_error, lin_error]
    
    def update(self, current):

        error_in_world_frame = self.error(current)
        self._sum_error = self._sum_error + error_in_world_frame * self._dt
        return self._Kp * error_in_world_frame + self._Ki * self._sum_error