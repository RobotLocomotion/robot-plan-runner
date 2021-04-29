import time
import threading

import zmq
import lcm

from make_joint_space_trajectory_plan import *
from drake import lcmt_iiwa_status


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

        self.iiwa_position_getter = IiwaPositionGetter()

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
                print("wait for result status", self.status)
                break
            time.sleep(0.01)
        print("Final status:", self.status)

    def abort(self):
        self.abort_client.send(b"abort")
        s = self.abort_client.recv_string()
        print(s)


zmq_client = PlanManagerZmqClient()
#%%
duration = 5.0

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
    zmq_client.make_and_send_plan([0, 6], q_knots_return)
    print("Returning to q0.")
    # TODO: sometimes, wait_for_result would return the status of the
    #  previous plan. We may need to return a signature of the plan together
    #  with the stauts in the PLAN_STATUS channel.
    zmq_client.wait_for_result()
    print("returned to q0.")
    print("------------------------------------------------------")


