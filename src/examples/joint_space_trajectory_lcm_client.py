import lcm
from make_joint_space_trajectory_plan import *

# This script shows how to send plan messages over LCM to
#  IiwaPlanManagerSystem, the drake systems wrapper of IiwaPlanManager.

lc = lcm.LCM()

#%% send msg 1 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots1).encode())

#%% send msg 2 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots2).encode())
