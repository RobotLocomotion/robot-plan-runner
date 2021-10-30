import time

import numpy as np
from drake import lcmt_robot_state
from drake import lcmt_robot_plan


def calc_joint_space_plan_msg(t_knots, q_knots):
    n_knots, n_q = q_knots.shape
    msg_plan = lcmt_robot_plan()

    # It is important that the utime of each plan is unique. The drake
    # systems version of PlanManager uses utime to tell if one plan is
    # different from another.
    msg_plan.utime = round(time.time() * 1000)

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


def calc_task_space_plan_msg(X_ET, X_WT_list, t_knots):
    n_knots = len(t_knots)
    msg_plan = lcmt_robot_plan()
    msg_plan.utime = round(time.time() * 1000)
    msg_plan.num_states = n_knots + 1
    joint_names = ["qw", "qx", "qy", "qz", "px", "py", "pz"]
    n_q = len(joint_names)

    # TODO: use our own lcm types.

    # The first n_knots msg_states are for the trajectory.
    for i, X_WTi in enumerate(X_WT_list):
        msg_state = lcmt_robot_state()
        msg_state.utime = int(1e6 * t_knots[i])
        msg_state.num_joints = n_q
        msg_state.joint_name = joint_names
        msg_state.joint_position = np.hstack(
            [X_WTi.rotation().ToQuaternion().wxyz(), X_WTi.translation()])

        msg_plan.plan.append(msg_state)

    # The last state in msg_plan encodes the offset between tool frame (
    # T) and EE frame E.
    msg_state_ET = lcmt_robot_state()
    msg_state_ET.utime = 0
    msg_state_ET.num_joints = 7
    msg_state_ET.joint_name = joint_names
    msg_state_ET.joint_position = np.hstack(
        [X_ET.rotation().ToQuaternion().wxyz(), X_ET.translation()])
    msg_plan.plan.append(msg_state_ET)

    return msg_plan

def calc_squeegee_plan_msg(X_ET, X_WT_list, t_knots):
    n_knots = len(t_knots)
    msg_plan = lcmt_robot_plan()
    msg_plan.utime = round(time.time() * 1000)
    msg_plan.num_states = n_knots + 1
    joint_names = ["qw_squeegee", "qx", "qy", "qz", "px", "py", "pz"]
    n_q = len(joint_names)

    # TODO: use our own lcm types.

    # The first n_knots msg_states are for the trajectory.
    for i, X_WTi in enumerate(X_WT_list):
        msg_state = lcmt_robot_state()
        msg_state.utime = int(1e6 * t_knots[i])
        msg_state.num_joints = n_q
        msg_state.joint_name = joint_names
        msg_state.joint_position = np.hstack(
            [X_WTi.rotation().ToQuaternion().wxyz(), X_WTi.translation()])

        msg_plan.plan.append(msg_state)

    # The last state in msg_plan encodes the offset between tool frame (
    # T) and EE frame E.
    msg_state_ET = lcmt_robot_state()
    msg_state_ET.utime = 0
    msg_state_ET.num_joints = 7
    msg_state_ET.joint_name = joint_names
    msg_state_ET.joint_position = np.hstack(
        [X_ET.rotation().ToQuaternion().wxyz(), X_ET.translation()])
    msg_plan.plan.append(msg_state_ET)

    return msg_plan
