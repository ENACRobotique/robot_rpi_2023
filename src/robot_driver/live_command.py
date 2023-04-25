import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

ecal_core.initialize(sys.argv, "cmd_live_test")
time.sleep(0.5)

pos_pub = ProtoPublisher('set_position',robot_pb.Position)
slow_pub = ProtoPublisher("slow",robot_pb.no_args_func_)
stop_pub = ProtoPublisher("stop",robot_pb.no_args_func_)
resume_pub = ProtoPublisher("resume",robot_pb.no_args_func_)
def bouge(x,y, theta):
    pos_pub.send(robot_pb.Position(x=x,y=y,theta=theta))

def slow():    
    print("slowing")
    slow_pub.send(robot_pb.no_args_func_(nothing = 1))

def stop():
    print("stoped")
    stop_pub.send(robot_pb.no_args_func_(nothing = 1))

def resume():
    print("resumed")
    resume_pub.send(robot_pb.no_args_func_(nothing = 1))
    
def end():
    ecal_core.finalize()
