import numpy as np
import zmq
import lcm
import time, copy

from drake import lcmt_robot_state
from drake import lcmt_robot_plan

from plan_runner_client.zmq_client import PlanManagerZmqClient
from plan_runner_client.calc_plan_msg import (
    calc_task_space_plan_msg,
    calc_joint_space_plan_msg
)

from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.common.eigen_geometry import Quaternion
from pydrake.trajectories import PiecewisePolynomial

def normalized(x):
    return x / np.linalg.norm(x)

zmq_client = PlanManagerZmqClient()
frame_E = zmq_client.plant.GetFrameByName('iiwa_link_7')
X_ET = RigidTransform(RollPitchYaw(0, -np.pi/2,0), np.array([0, 0, 0.255]))

# Step 1. Check that the correct Cartesian position is valid and kinematics are working correctly.
q_now = zmq_client.get_current_joint_angles()
X_WE = zmq_client.get_current_ee_pose(frame_E)
print("Current joint angles:")
print(q_now)
print("Current end-effector position:")
print(X_WE)
time.sleep(1.0)


# Step 2. Test every joint.
print("Testing joint-space plan by joint flexing.")
duration = 50
t_knots = np.linspace(0, duration, 22)
q_knots = np.zeros((22, 7))

q_knots[0,:] = zmq_client.get_current_joint_angles()
for i in range(7):
    q_knots[3*i+1,:] = q_knots[3*i,:]
    q_knots[3*i+1,i] += 0.2
    q_knots[3*i+2,:] = q_knots[3*i+1,:]
    q_knots[3*i+2,i] -= 0.4
    q_knots[3*i+3,:] = q_knots[3*i+2,:]
    q_knots[3*i+3,i] += 0.2

print(q_knots)

plan_msg1 = calc_joint_space_plan_msg(t_knots, q_knots)
zmq_client.send_plan(plan_msg1)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()

time.sleep(2.0)


# Step 3. Go to basic position for task-space demo.
duration = 3
t_knots = np.linspace(0, duration, 2)
X_WT_lst = []
X_WT_lst.append(zmq_client.get_current_ee_pose(frame_E))
X_WT_lst.append(RigidTransform(RollPitchYaw([0, -np.pi, 0]), np.array([0.5, 0.0, 0.5])))

plan_msg2 = calc_task_space_plan_msg(RigidTransform(), X_WT_lst, t_knots)
zmq_client.send_plan(plan_msg2)
zmq_client.wait_for_plan_to_finish()

# Step 4. Test Cartesian flexing.
duration = 50
t_knots = np.linspace(0, duration, 19)
q_knots = np.zeros((19, 6))

q_knots[0,:] = np.array([0, -np.pi, 0, 0.5, 0.0, 0.5])

for i in range(6):
    q_knots[3*i+1,:] = q_knots[3*i,:]
    q_knots[3*i+1,i] += 0.1
    q_knots[3*i+2,:] = q_knots[3*i+1,:]
    q_knots[3*i+2,i] -= 0.2
    q_knots[3*i+3,:] = q_knots[3*i+2,:]
    q_knots[3*i+3,i] += 0.1

X_WT_lst = []
for i in range(19):
    X_WT_lst.append(RigidTransform(
        RollPitchYaw(q_knots[i,0:3]), q_knots[i,3:6]))

plan_msg3 = calc_task_space_plan_msg(RigidTransform(), X_WT_lst, t_knots)
zmq_client.send_plan(plan_msg3)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()
