import time
import numpy as np

from pydrake.all import DrakeLcm, RotationMatrix
from pydrake.math import RollPitchYaw
from drake import lcmt_robot_state

channel_name = "X_TC"
lcm = DrakeLcm()

t0 = time.time()
while True:
    msg = lcmt_robot_state()
    t = time.time()
    msg.utime = int(t * 1e6)
    msg.num_joints = 7
    msg.joint_name = ["qw", "qx", "qy", "qz", "px", "py", "pz"]
    R_TC = RollPitchYaw(0, 0, 0.2 * np.sin(t - t0)).ToRotationMatrix()
    msg.joint_position = np.hstack([R_TC.ToQuaternion().wxyz(), np.zeros(3)])
    lcm.Publish(channel_name, msg.encode())
    time.sleep(0.01)

