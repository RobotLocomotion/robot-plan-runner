import time
import threading

import numpy as np
import zmq
import lcm

from drake import lcmt_robot_state
from drake import lcmt_robot_plan

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


#%% zmq client.
class PlanManagerZmqClient:
    def __init__(self):
        self.context = zmq.Context()
        self.plan_client = self.context.socket(zmq.REQ)
        self.plan_client.connect("tcp://localhost:5555")

        self.status_subscriber = self.context.socket(zmq.SUB)
        self.status_subscriber.setsockopt_string(zmq.SUBSCRIBE, "PLAN_STATUS")
        # Setting zmq.CONFLATE to true should keep only the last message,
        # but in practice zmq seems to only discard messages older than a
        # finite amount of time (a fraction of a second is what I observed).
        self.status_subscriber.setsockopt(zmq.CONFLATE, True)
        self.status_subscriber.connect("tcp://localhost:5557")
        self.status = None
        self.last_status_time = None
        self.t1 = threading.Thread(target=self.subscribe_to_status, daemon=True)
        self.t1.start()

        self.abort_client = self.context.socket(zmq.REQ)
        self.abort_client.connect("tcp://localhost:5556")

    def subscribe_to_status(self):
        while True:
            s = self.status_subscriber.recv_string()
            channel, status = s.split()
            self.status = status
            self.last_status_time = time.time()

    def make_and_send_plan(self, t_knots, q_knots):
        self.plan_client.send(calc_plan_msg(t_knots, q_knots).encode())
        msg = self.plan_client.recv()
        assert msg == b'plan_received'
        print("plan received by server.")

    def wait_for_result(self):
        while True:
            if self.status != "running":
                break
            time.sleep(0.01)
        print("Final status:", self.status)

    def abort(self):
        self.abort_client.send(b"abort")
        s = self.abort_client.recv_string()
        print(s)


zmq_client = PlanManagerZmqClient()
#%%
zmq_client.make_and_send_plan([0, 6], q_knots1)
print("pretending to do some work")
time.sleep(3.0)
# zmq_client.abort()
zmq_client.wait_for_result()
print("plan finished.")

#%%
# test waiting longer than plan duration.
zmq_client.make_and_send_plan([0, 5], q_knots2)
print("pretending to do some work")
time.sleep(6.0)
print("waiting for result")
zmq_client.wait_for_result()


#%% lcm client.
lc = lcm.LCM()

#%% send msg 1 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots1).encode())

#%% send msg 2 via lcm.
lc.publish("ROBOT_PLAN", calc_plan_msg(t_knots, q_knots2).encode())
