import time

import numpy as np
from plan_runner_client.zmq_client import PlanManagerZmqClient
from drake import lcmt_robot_plan, lcmt_robot_state

zmq_client = PlanManagerZmqClient()

frame_L7 = zmq_client.plant.GetFrameByName('iiwa_link_7')
n_q = 7
X_WL7 = zmq_client.get_ee_pose_commanded(frame_L7)

#%%
joint_names = ["qw_chicken", "qx_chicken", "qy_chicken", "qz_chicken", "px_chicken",
               "py_chicken", "pz_chicken"]
t_knots = [0, 5.]

msg_plan = lcmt_robot_plan()
msg_plan.utime = round(time.time() * 1000)
msg_plan.num_states = 2

for i in range(len(t_knots)):
    msg_state = lcmt_robot_state()
    msg_state.utime = int(1e6 * t_knots[i])
    msg_state.num_joints = n_q
    msg_state.joint_name = joint_names
    msg_state.joint_position = np.hstack(
        [X_WL7.rotation().ToQuaternion().wxyz(), X_WL7.translation()])
    msg_plan.plan.append(msg_state)

#%%
zmq_client.send_plan(msg_plan)
zmq_client.wait_for_plan_to_finish()
