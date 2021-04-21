import numpy as np
import zmq
import lcm

from drake import lcmt_robot_state
from drake import lcmt_robot_plan

from pydrake.trajectories import PiecewisePolynomial

#%%
t_knots = np.array([0, 10])

q_knots1 = np.zeros((2, 7))
# These numbers are set to be approximately starting from the EE
# position in mock_station_simulation.

q_knots1[0,:] = [0., 0., 1., 0., 0.5, 0.0, 0.3]
q_knots1[1,:] = [0., 0., 1., 0., 0.5, 0.2, 0.3]

q_knots2 = np.zeros((2, 7))
q_knots2[0,:] = [0., 0., 1., 0., 0.5, 0.2, 0.3]
q_knots2[1,:] = [0.718, 0., 0.696, 0., 0.4, 0.0, 0.3]

def calc_plan_msg(t_knots, q_knots):
    n_knots, n_q = q_knots.shape
    msg_plan = lcmt_robot_plan()

    joint_names = ["qw", "qx", "qy", "qz", "px", "py", "pz"]

    msg_plan.num_states = n_knots
    for i, q_i in enumerate(q_knots):
        msg_state = lcmt_robot_state()
        msg_state.utime = int(1e6 * t_knots[i])
        msg_state.num_joints = n_q
        msg_state.joint_name = joint_names
        msg_state.joint_position = q_i

        msg_plan.plan.append(msg_state)

    return msg_plan

#%% zmq client
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#%% send msg 1 via zmq.
socket.send(calc_plan_msg(t_knots, q_knots1).encode())
print("lcm msg sent")
msg = socket.recv()
print(msg)

#%% send msg 2 via zmq.
socket.send(calc_plan_msg(t_knots, q_knots2).encode())
print("lcm msg sent")
msg = socket.recv()
print(msg)


#%% lcm client.
lc = lcm.LCM()

#%% send msg 1 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots1).encode())

#%% send msg 2 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots2).encode())
