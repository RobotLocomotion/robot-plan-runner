import time
import argparse

import numpy as np
from pydrake.all import RigidTransform

from plan_runner_client.calc_plan_msg import (
    calc_joint_space_plan_msg,
    calc_task_space_plan_msg
)
from plan_runner_client.zmq_client import PlanManagerZmqClient

zmq_client = PlanManagerZmqClient()

t_knots = np.array([0, 10])
q0 = zmq_client.get_current_joint_angles()

if len(q0) == 0:
    raise RuntimeError("No messages were detected in IIWA_STATUS. " +
                       "Is the simulation runnning?")

q_knots1 = np.zeros((2, 7))
q_knots1[:, ] = q0
q_knots1[1, 0] += 1

q_knots2 = np.zeros((2, 7))
q_knots2[0] = q_knots1[1]
q_knots2[1] = q_knots1[0]
duration = 5.0


def run_joint_space_plan():
    """Test joint space plan."""
    plan_msg = calc_joint_space_plan_msg([0, duration], q_knots1)
    zmq_client.send_plan(plan_msg)
    time.sleep(1.0)
    zmq_client.wait_for_plan_to_finish()


def run_joint_space_plan_abort():
    """Test joint space plan with abort."""
    plan_msg = calc_joint_space_plan_msg([0, duration], q_knots1)
    zmq_client.send_plan(plan_msg)
    time.sleep(3.0)
    # Calling wait for plan to finish immediately after calling abort should
    #  return FINISHED.
    zmq_client.abort()
    zmq_client.wait_for_plan_to_finish()


def run_task_space_plan():
    """Test task space plan."""
    frame_E = zmq_client.plant.GetFrameByName('iiwa_link_7')
    X_ET = RigidTransform()
    X_ET.set_translation([0.1, 0, 0])
    X_WE0 = zmq_client.get_current_ee_pose(frame_E)
    X_WT0 = X_WE0.multiply(X_ET)
    X_WT1 = RigidTransform(X_WT0.rotation(),
                           X_WT0.translation() + np.array([0, 0.2, 0]))
    plan_msg = calc_task_space_plan_msg(X_ET, [X_WT0, X_WT1], [0, 5])
    zmq_client.send_plan(plan_msg)


def run_joint_space_plan_loop():
    """
    Send random joint space plan, randomly interrupt, return to home,
    and start over. 
    """
    # TODO: support WaitForServer, which blocks until the server is in state
    #  IDLE.
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


def generate_doc_string(test_lst):
    doc_string = "Input an integer corresponding to the test.\n"
    for i in range(len(test_lst)):
        doc_string += str(i) + ": "
        doc_string += "[" + test_lst[i].__name__ + "] : "
        doc_string += test_lst[i].__doc__
        doc_string += "\n"
    return doc_string


if __name__ == "__main__":
    # New tests should be added in the list.
    test_lst = [
        run_joint_space_plan,
        run_joint_space_plan_abort,
        run_task_space_plan,
        run_joint_space_plan_loop,
    ]

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("test_type", type=int,
                        help=generate_doc_string(test_lst))

    args = parser.parse_args()
    test_lst[args.test_type]()
