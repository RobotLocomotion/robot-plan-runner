import lcm
from make_joint_space_trajectory_plan import *

#%% lcm client.
lc = lcm.LCM()

#%% send msg 1 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots1).encode())

#%% send msg 2 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots2).encode())
