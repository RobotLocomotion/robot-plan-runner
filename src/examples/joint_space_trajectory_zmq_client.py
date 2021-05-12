import time
import threading
import sys

import zmq
import lcm

from make_joint_space_trajectory_plan import *
from drake import lcmt_iiwa_status

from robot_plan_runner import lcmt_plan_status, lcmt_plan_status_constants

#%% zmq client.
class IiwaPositionGetter:
    def __init__(self):
        self.lc = lcm.LCM()
        sub = self.lc.subscribe("IIWA_STATUS", self.sub_callback)
        sub.set_queue_capacity(1)
        self.iiwa_position_measured = None
        self.t1 = threading.Thread(target=self.update_iiwa_position_measured)
        self.t1.start()

    def sub_callback(self, channel, data):
        iiwa_status_msg = lcmt_iiwa_status.decode(data)
        self.iiwa_position_measured = iiwa_status_msg.joint_position_measured

    def update_iiwa_position_measured(self):
        while True:
            self.lc.handle()

    def get_iiwa_position_measured(self):
        return np.array(self.iiwa_position_measured)


class PlanManagerZmqClient:
    def __init__(self):
        self.context = zmq.Context()
        self.plan_client = self.context.socket(zmq.REQ)
        self.plan_client.connect("tcp://localhost:5555")

        self.status_subscriber = self.context.socket(zmq.SUB)
        self.channel_name = "PLAN_STATUS"
        self.status_subscriber.setsockopt_string(zmq.SUBSCRIBE,
                                                 self.channel_name)
        self.last_plan_msg = lcmt_robot_plan()
        self.last_plan_msg.utime = sys.maxsize  # 2**63 - 1
        self.plan_msg_lock = threading.Lock()
        # Setting zmq.CONFLATE to true should keep only the last message,
        # but in practice zmq seems to only discard messages older than a
        # finite amount of time (a fraction of a second is what I observed).
        self.status_subscriber.setsockopt(zmq.CONFLATE, True)
        self.status_subscriber.connect("tcp://localhost:5557")
        self.last_status_msg = None
        self.status_msg_lock = threading.Lock()
        self.t1 = threading.Thread(target=self.subscribe_to_status, daemon=True)
        self.t1.start()

        self.abort_client = self.context.socket(zmq.REQ)
        self.abort_client.connect("tcp://localhost:5556")

        self.iiwa_position_getter = IiwaPositionGetter()

    def subscribe_to_status(self):
        while True:
            msg = self.status_subscriber.recv()
            lcm_msg_bytes = msg[len(self.channel_name) + 1:]
            lcm_msg = lcmt_plan_status.decode(lcm_msg_bytes)
            self.status_msg_lock.acquire()
            self.last_status_msg = lcm_msg
            self.status_msg_lock.release()

    def make_and_send_plan(self, t_knots, q_knots):
        self.plan_msg_lock.acquire()
        self.last_plan_msg = calc_plan_msg(t_knots, q_knots)
        self.plan_msg_lock.release()
        self.plan_client.send(self.last_plan_msg.encode())
        msg = self.plan_client.recv()
        assert msg == b'plan_received'
        print("plan received by server.")

    def wait_for_plan_to_finish(self):
        while True:
            self.status_msg_lock.acquire()
            self.plan_msg_lock.acquire()
            is_same_plan = (
                self.last_plan_msg.utime == self.last_status_msg.utime)
            is_plan_finished = (
                self.last_status_msg.status ==
                lcmt_plan_status_constants.FINISHED)
            self.plan_msg_lock.release()
            self.status_msg_lock.release()

            if is_same_plan and is_plan_finished:
                break
            time.sleep(0.01)
        print("Final status:", self.last_status_msg.status)

    def abort(self):
        self.abort_client.send(b"abort")
        s = self.abort_client.recv_string()
        print(s)


zmq_client = PlanManagerZmqClient()

#%%
duration = 5.0
zmq_client.make_and_send_plan([0, duration], q_knots1)
time.sleep(1.0)
zmq_client.wait_for_plan_to_finish()

zmq_client.make_and_send_plan([0, duration], q_knots2)
time.sleep(1.0)
# Calling wait for plan to finish immediately after calling abort should
#  return FINISHED.
zmq_client.abort()
zmq_client.wait_for_plan_to_finish()

#%%
# TODO: support WaitForServer, which blocks until the server is in state IDLE.
while True:
    print("plan sent")
    zmq_client.make_and_send_plan([0, duration], q_knots1)
    print("pretending to do some work")
    stop_duration = np.random.rand() * duration
    time.sleep(stop_duration)
    zmq_client.abort()
    print("plan aborted after t = {}s".format(stop_duration))

    # # get current robot position.
    q_now = zmq_client.iiwa_position_getter.get_iiwa_position_measured()
    q_knots_return = np.vstack([q_now, q0])
    zmq_client.make_and_send_plan([0, stop_duration], q_knots_return)
    print("Returning to q0.")
    zmq_client.wait_for_plan_to_finish()
    print("returned to q0.")
    print("------------------------------------------------------")


