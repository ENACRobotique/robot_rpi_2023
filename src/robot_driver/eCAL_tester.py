import robot_state_pb2 as robot_pb
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher

from time import sleep

ecal_core.initialize([], "tester")

pub = ProtoPublisher("set_position", robot_pb.Position)

sleep(1.0)

msg = robot_pb.Position(x=float(1.1), y=float(2.0), theta=float(3.0))
pub.send(msg)

sleep(0.01)

msg = robot_pb.Position(x=float(2.1), y=float(1.0), theta=float(3.0))
pub.send(msg)

sleep(0.01)
msg = robot_pb.Position(x=float(3.1), y=float(1.0), theta=float(3.0))
pub.send(msg)

sleep(0.01)

msg = robot_pb.Position(x=float(4.1), y=float(1.0), theta=float(3.0))
pub.send(msg)

sleep (1.0)

ecal_core.finalize()


