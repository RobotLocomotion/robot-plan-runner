import lcm
import numpy as np

from plan_runner_client.zmq_client import IiwaPositionGetter
from plan_runner_client.calc_plan_msg import (
    calc_joint_space_plan_msg,
    calc_task_space_plan_msg
)

# This script shows how to send plan messages over LCM to
#  IiwaPlanManagerSystem, the drake systems wrapper of IiwaPlanManager.

lc = lcm.LCM()

# This is imported just to query for current joint angles.
iiwa_position_getter = IiwaPositionGetter()

#%% Tests joint space plan.
t_knots = np.array([0, 10])

# This assumes that the simulation is running and lcm messages are being
# published in IIWA_STATUS.
q0 = iiwa_position_getter.get_iiwa_position_measured()
if (q0[0] == None):
    raise RuntimeError("No messages were detected in IIWA_STATUS. " + 
                     "Is the simulation runnning?")    
q_knots1 = np.zeros((2, 7))
q_knots1[:, ] = q0
q_knots1[1, 0] += 1

q_knots2 = np.zeros((2, 7))
q_knots2[0] = q_knots1[1]
q_knots2[1] = q_knots1[0]

#%% send msg 1 via lcm.
lc.publish("ROBOT_PLAN", calc_joint_space_plan_msg(t_knots, q_knots1).encode())

#%% send msg 2 via lcm.
lc.publish("ROBOT_PLAN", calc_joint_space_plan_msg(t_knots, q_knots2).encode())
