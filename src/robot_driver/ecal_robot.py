import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import robot_state_pb2 as robot_pb
import comm_old as comm

class EcalRadio(comm.Radio):
    def __init__(self):
        super().__init__()
        print("serial initialized - waiting for ecal")
        ecal_core.initialize(sys.argv, "robot_driver")
        self.odom_position = ProtoPublisher('odom_pos', robot_pb.Position)
        time.sleep(1.0) # time needed to initialize ecal
        print("ecal initialized too !")


    ####  Reception from robot
    def on_odom_position(self, x, y, theta):
        self.odom_position.send(robot_pb.Position(x=x, y=y, theta=theta))


def init_ecal(self):
    """Initialize eCAL"""
    ecal_core.initialize(sys.argv, "robot_driver")
    
    # Create the subscribers & set the callbacks
    ProtoSubscriber("set_position", robot_pb.Position).set_callback(self.on_set_position)
    ProtoSubscriber("reach_position", robot_pb.Position).set_callback(self.on_reach_position)
    StringSubscriber("stop_cons").set_callback(self.on_stop_cons)
    ProtoSubscriber("set_pince", robot_pb.SetState).set_callback(self.on_pince)
    ProtoSubscriber("pick_disk", robot_pb.SetState).set_callback(self.on_pick_disk)
    ProtoSubscriber("drop_disk", robot_pb.SetState).set_callback(self.on_drop_disk)
    ProtoSubscriber("set_turbine", robot_pb.SetState).set_callback(self.on_turbine)
    ProtoSubscriber("set_toboggan", robot_pb.SetState).set_callback(self.on_toboggan)
    ProtoSubscriber("set_score", robot_pb.SetState).set_callback(self.on_score)
    ProtoSubscriber("end_match", robot_pb.Bool).set_callback(self.on_end_match)
    