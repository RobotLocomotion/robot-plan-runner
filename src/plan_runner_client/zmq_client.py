import time
import threading
import sys
import copy

import zmq
import lcm

import numpy as np

from pydrake.all import MultibodyPlant, Parser, RigidTransform
from pydrake.common import FindResourceOrThrow
from drake import (
    lcmt_iiwa_status, lcmt_robot_plan,
    lcmt_schunk_wsg_status, lcmt_schunk_wsg_command)

from robot_plan_runner import lcmt_plan_status, lcmt_plan_status_constants


def build_iiwa7_plant():
    plant = MultibodyPlant(1e-3)
    parser = Parser(plant=plant)

    iiwa_drake_path = (
        "drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf")
    iiwa_path = FindResourceOrThrow(iiwa_drake_path)
    robot_model = parser.AddModelFromFile(iiwa_path)

    # weld robot to world frame.
    plant.WeldFrames(frame_on_parent_P=plant.world_frame(),
                     frame_on_child_C=plant.GetFrameByName("iiwa_link_0"),
                     X_PC=RigidTransform.Identity())
    plant.Finalize()

    return plant


class IiwaPositionGetter:
    def __init__(self):
        self.lc = lcm.LCM()
        sub = self.lc.subscribe("IIWA_STATUS", self.sub_callback)
        sub.set_queue_capacity(1)
        self.iiwa_position_measured = None
        self.msg_lock = threading.Lock()
        self.t1 = threading.Thread(target=self.update_iiwa_position_measured)
        self.t1.start()

    def sub_callback(self, channel, data):
        iiwa_status_msg = lcmt_iiwa_status.decode(data)
        self.msg_lock.acquire()
        self.iiwa_position_measured = np.array(
            iiwa_status_msg.joint_position_measured)
        self.msg_lock.release()

    def update_iiwa_position_measured(self):
        while True:
            self.lc.handle()

    def get_iiwa_position_measured(self):
        iiwa_position_measured = np.array([])
        self.msg_lock.acquire()
        if self.iiwa_position_measured:
            iiwa_position_measured = np.array(self.iiwa_position_measured)
        self.msg_lock.release()
        return iiwa_position_measured


class PlanManagerZmqClient:
    def __init__(self):
        self.context = zmq.Context()
        self.plan_client = self.context.socket(zmq.REQ)
        # TODO: load sockets from config.yml.
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

        # subscribe to IIWA_STATUS.
        self.iiwa_position_getter = IiwaPositionGetter()

        # iiwa7 plant.
        self.plant = build_iiwa7_plant()

    def subscribe_to_status(self):
        while True:
            msg = self.status_subscriber.recv()
            lcm_msg_bytes = msg[len(self.channel_name) + 1:]
            lcm_msg = lcmt_plan_status.decode(lcm_msg_bytes)
            self.status_msg_lock.acquire()
            self.last_status_msg = lcm_msg
            self.status_msg_lock.release()

    def get_plan_status(self):
        self.status_msg_lock.acquire()
        status_msg = copy.deepcopy(self.last_status_msg)
        self.status_msg_lock.release()
        return status_msg

    def get_current_ee_pose(self, frame_E):
        context = self.plant.CreateDefaultContext()
        q = self.iiwa_position_getter.get_iiwa_position_measured()
        self.plant.SetPositions(context, q)
        return self.plant.CalcRelativeTransform(
            context, self.plant.world_frame(), frame_E)

    def get_current_joint_angles(self):
        return self.iiwa_position_getter.get_iiwa_position_measured()

    def send_plan(self, plan_msg):
        self.plan_msg_lock.acquire()
        self.last_plan_msg = copy.deepcopy(plan_msg)
        self.plan_msg_lock.release()
        self.plan_client.send(plan_msg.encode())
        msg = self.plan_client.recv()
        assert msg == b'plan_received'
        print("plan received by server.")

    def wait_for_plan_to_finish(self):
        # TODO: add timeout.
        while True:
            status_msg = self.get_plan_status()
            self.plan_msg_lock.acquire()
            is_same_plan = (
                self.last_plan_msg.utime == status_msg.utime)
            is_plan_finished = (
                status_msg.status ==
                lcmt_plan_status_constants.FINISHED)
            is_plan_error = (
                status_msg.status ==
                lcmt_plan_status_constants.ERROR)
            self.plan_msg_lock.release()

            if is_same_plan and (is_plan_finished or is_plan_error):
                break
            time.sleep(0.01)
        print("Final status:", status_msg.status)

    def abort(self):
        self.abort_client.send(b"abort")
        s = self.abort_client.recv_string()
        print(s)


class SchunkManager:
    def __init__(self, force_limit=40.0):
        self.lc = lcm.LCM()
        sub = self.lc.subscribe("SCHUNK_WSG_STATUS", self.sub_callback)
        sub.set_queue_capacity(1)
        self.schunk_position_measured = None
        self.schunk_position_commanded = None
        self.msg_lock = threading.Lock()
        self.t1 = threading.Thread(target=self.update_schunk_position_measured)
        self.t1.start()
        self.force_limit = force_limit

    def sub_callback(self, channel, data):
        schunk_status_msg = lcmt_schunk_wsg_status.decode(data)
        self.msg_lock.acquire()
        self.schunk_position_measured = schunk_status_msg.actual_position_mm
        self.msg_lock.release()

    def update_schunk_position_measured(self):
        while True:
            self.lc.handle()

    def get_schunk_position_measured(self):
        self.msg_lock.acquire()
        p = np.array(self.schunk_position_measured)
        self.msg_lock.release()
        return p

    def send_schunk_position_command(self, command_mm):
        self.schunk_position_commanded = command_mm
        msg = lcmt_schunk_wsg_command()
        msg.utime = 0 # ignore time?
        msg.target_position_mm = command_mm
        msg.force = self.force_limit
        self.lc.publish("SCHUNK_WSG_COMMAND", msg.encode())

    def wait_for_command_to_finish(self):
        # NOTE(terry-suh): Note that when doing a grasp, the commanded position
        # typically penetrates the object, and therefore the actual position 
        # will never reach the desired position, causing this method to hang
        # indefinitely. In this case, it is more advisable to simply wait for 
        # a set time using time.sleep instead of calling this method.
        time_now = time.time()
        while True:
            reached_goal = (
                np.abs(self.get_schunk_position_measured() -
                       self.schunk_position_commanded) < 0.5)
            if reached_goal:
                print("Schunk command is successfully executed.")
                break
            if (time.time() - time_now) < 60.0:
                print("Timeout. 60 seconds elapsed but Schunk failed to reach.")
                break
