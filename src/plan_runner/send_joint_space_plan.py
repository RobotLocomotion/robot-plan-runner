import numpy as np
import zmq

from drake import lcmt_robot_state
from drake import lcmt_robot_plan

from pydrake.trajectories import PiecewisePolynomial

#%%
t_knots = np.array([0, 10])
q_knots1 = np.zeros((2, 7))
q_knots1[:, ] = [0, 0.6, 0, -1.75, 0, 1, 0]
q_knots1[1, 0] += 1

q_knots2 = np.zeros((2, 7))
q_knots2[0] = q_knots1[1]
q_knots2[1] = q_knots1[0]


def calc_plan_msg(t_knots, q_knots):
    n_knots, n_q = q_knots.shape
    msg_plan = lcmt_robot_plan()

    joint_names = ["iiwa_joint_{}".format(i) for i in range(n_q)]

    msg_plan.num_states = n_knots
    for i, q_i in enumerate(q_knots):
        msg_state = lcmt_robot_state()
        msg_state.utime = int(1e6 * t_knots[i])
        msg_state.num_joints = n_q
        msg_state.joint_name = joint_names
        msg_state.joint_position = q_i

        msg_plan.plan.append(msg_state)

    return msg_plan


#%%
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#%%
socket.send(calc_plan_msg(t_knots, q_knots1).encode())
print("lcm msg sent")
msg = socket.recv()
print(msg)

#%%
socket.send(calc_plan_msg(t_knots, q_knots2).encode())
print("lcm msg sent")
msg = socket.recv()
print(msg)
