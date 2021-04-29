import time
import threading

import zmq
import lcm

from make_joint_space_trajectory_plan import *
from drake import lcmt_iiwa_status

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

#%%

class IiwaPositionGetter:
    def __init__(self):
        self.lc = lcm.LCM()
        self.lc.subscribe("IIWA_STATUS", self.sub_callback)
        self.iiwa_position_measured = None

    def sub_callback(self, channel, data):
        iiwa_status_msg = lcmt_iiwa_status.decode(data)
        self.iiwa_position_measured = iiwa_status_msg.joint_position_measured

    def get_iiwa_position_measured(self):
        self.lc.handle()
        return self.iiwa_position_measured

