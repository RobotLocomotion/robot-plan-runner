import numpy as np
import zmq
import lcm
import time, copy

from drake import lcmt_robot_state
from drake import lcmt_robot_plan

from plan_runner_client.zmq_client import PlanManagerZmqClient, SchunkManager
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
schunk = SchunkManager()
frame_E = zmq_client.plant.GetFrameByName('iiwa_link_7')
X_ET = RigidTransform(RollPitchYaw(0, -np.pi / 2, 0), np.array([0, 0, 0.255]))

# Step 1. Check that the correct Cartesian position is valid and kinematics
# are working correctly.
q_now = zmq_client.get_current_joint_angles()
X_WE = zmq_client.get_current_ee_pose(frame_E)
print("Current joint angles:")
print(q_now)
print("Current end-effector position:")
print(X_WE)
time.sleep(1.0)

schunk.send_schunk_position_command(25)
schunk.wait_for_command_to_finish()
time.sleep(5.0)
schunk.send_schunk_position_command(50)
schunk.wait_for_command_to_finish()
time.sleep(5.0)
schunk.send_schunk_position_command(0)
schunk.wait_for_command_to_finish()
time.sleep(5.0)

# Step 2. Test every joint.
print("Testing joint-space plan by joint flexing.")
duration = 30
t_knots = np.linspace(0, duration, 22)
q_knots = np.zeros((22, 7))

q_knots[0, :] = zmq_client.get_current_joint_angles()
for i in range(7):
    q_knots[3 * i + 1, :] = q_knots[3 * i, :]
    q_knots[3 * i + 1, i] += 0.2
    q_knots[3 * i + 2, :] = q_knots[3 * i + 1, :]
    q_knots[3 * i + 2, i] -= 0.4
    q_knots[3 * i + 3, :] = q_knots[3 * i + 2, :]
    q_knots[3 * i + 3, i] += 0.2

plan_msg1 = calc_joint_space_plan_msg(t_knots, q_knots)
zmq_client.send_plan(plan_msg1)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()
time.sleep(2.0)

# Step 3. Go to basic position for task-space demo.
duration = 2
t_knots = np.linspace(0, duration, 2)
q_knots = np.zeros((2, 7))

q_knots[0, :] = zmq_client.get_current_joint_angles()
q_knots[1, :] = [0, 0.3, 0.0, -1.75, 0.0, 1.0, 0.0]

plan_msg2 = calc_joint_space_plan_msg(t_knots, q_knots)
zmq_client.send_plan(plan_msg2)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()

time.sleep(2.0)

# Step 4. Go to basic position for task-space demo.
duration = 2
t_knots = np.linspace(0, duration, 2)

X_WT_lst = []
X_WT_lst.append(zmq_client.get_current_ee_pose(frame_E))
X_WT_lst.append(RigidTransform(RollPitchYaw([0, -np.pi, 0]),
                               np.array([0.5, 0.0, 0.5])))

plan_msg2 = calc_task_space_plan_msg(RigidTransform(), X_WT_lst, t_knots)
zmq_client.send_plan(plan_msg2)
zmq_client.wait_for_plan_to_finish()

# Step 4. Test Cartesian flexing.
duration = 40
div = 5
t_knots = np.linspace(0, duration, 1 + div * 6)
q_knots = np.zeros((1 + div * 6, 6))

q_knots[0, :] = np.array([0, -np.pi, 0, 0.5, 0.0, 0.5])

for i in range(0, 3):
    q_knots[div * i + 1, :] = q_knots[div * i, :]
    q_knots[div * i + 1, i] += 0.3
    q_knots[div * i + 2, :] = q_knots[div * i + 1, :]
    q_knots[div * i + 3, :] = q_knots[div * i + 2, :]
    q_knots[div * i + 3, i] -= 0.6
    q_knots[div * i + 4, :] = q_knots[div * i + 3, :]
    q_knots[div * i + 5, :] = q_knots[div * i + 4, :]
    q_knots[div * i + 5, i] += 0.3

for i in range(3, 6):
    q_knots[div * i + 1, :] = q_knots[div * i, :]
    q_knots[div * i + 1, i] += 0.1
    q_knots[div * i + 2, :] = q_knots[div * i + 1, :]
    q_knots[div * i + 3, :] = q_knots[div * i + 2, :]
    q_knots[div * i + 3, i] -= 0.2
    q_knots[div * i + 4, :] = q_knots[div * i + 3, :]
    q_knots[div * i + 5, :] = q_knots[div * i + 4, :]
    q_knots[div * i + 5, i] += 0.1

X_WT_lst = []
for i in range(1 + div * 6):
    X_WT_lst.append(RigidTransform(
        RollPitchYaw(q_knots[i, 0:3]), q_knots[i, 3:6]))

plan_msg3 = calc_task_space_plan_msg(RigidTransform(), X_WT_lst, t_knots)
zmq_client.send_plan(plan_msg3)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()
