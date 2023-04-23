import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

ecal_core.initialize(sys.argv, "cmd_live_test")
time.sleep(0.5)

test_pub = ProtoPublisher('set_position',robot_pb.Position)

def bouge(x,y, theta):
    test_pub.send(robot_pb.Position(x=x,y=y,theta=theta))

def end():
    ecal_core.finalize()
