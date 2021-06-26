import time
import threading
import sys

import zmq
import lcm

import numpy as np

from pydrake.all import MultibodyPlant, Parser, RigidTransform
from pydrake.common import FindResourceOrThrow
from drake import lcmt_iiwa_status

from plan_runner_client.zmq_client import PlanManagerZmqClient
from plan_runner_client.calc_plan_msg import (
    calc_joint_space_plan_msg,
    calc_task_space_plan_msg
)

zmq_client = PlanManagerZmqClient()

#%% Tests joint space plan.
t_knots = np.array([0, 10])
q0 = zmq_client.get_current_joint_angles()
q_knots1 = np.zeros((2, 7))
q_knots1[:, ] = q0
q_knots1[1, 0] += 1

q_knots2 = np.zeros((2, 7))
q_knots2[0] = q_knots1[1]
q_knots2[1] = q_knots1[0]


#%% send some joint space plans.
print("1")
duration = 5.0
plan_msg = calc_joint_space_plan_msg([0, duration], q_knots1)
zmq_client.send_plan(plan_msg)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()

print("2")

plan_msg = calc_joint_space_plan_msg([0, duration], q_knots2)
zmq_client.send_plan(plan_msg)
time.sleep(1.0)
# Calling wait for plan to finish immediately after calling abort should
#  return FINISHED.
zmq_client.abort()
zmq_client.wait_for_plan_to_finish()


#%% send some task space plan.
frame_E = zmq_client.plant.GetFrameByName('iiwa_link_7')
X_ET = RigidTransform()
X_ET.set_translation([0.1, 0, 0])
X_WE0 = zmq_client.get_current_ee_pose(frame_E)
X_WT0 = X_WE0.multiply(X_ET)
X_WT1 = RigidTransform(X_WT0.rotation(),
                       X_WT0.translation() + np.array([0, 0.2, 0]))
plan_msg = calc_task_space_plan_msg(X_ET, [X_WT0, X_WT1], [0, 5])
zmq_client.send_plan(plan_msg)


#%% send some joint space plans, randomly interrupt, return to home,
# and start over.
# TODO: support WaitForServer, which blocks until the server is in state IDLE.
while True:
    print("plan sent")
    plan_msg = calc_joint_space_plan_msg([0, duration], q_knots1)
    zmq_client.send_plan(plan_msg)
    print("pretending to do some work")
    stop_duration = np.random.rand() * duration
    time.sleep(stop_duration)
    zmq_client.abort()
    print("plan aborted after t = {}s".format(stop_duration))

    # # get current robot position.
    q_now = zmq_client.iiwa_position_getter.get_iiwa_position_measured()
    q_knots_return = np.vstack([q_now, q0])
    plan_msg = calc_joint_space_plan_msg([0, stop_duration], q_knots_return)
    zmq_client.send_plan(plan_msg)
    print("Returning to q0.")
    zmq_client.wait_for_plan_to_finish()
    print("returned to q0.")
    print("------------------------------------------------------")
