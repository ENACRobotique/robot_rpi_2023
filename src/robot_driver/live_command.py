import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
from ..common.robot import*

ecal_core.initialize(sys.argv, "cmd_live_test")
time.sleep(0.5)

# pos_pub = ProtoPublisher('set_position',robot_pb.Position)
# slow_pub = ProtoPublisher("slow",robot_pb.no_args_func_)
# stop_pub = ProtoPublisher("stop",robot_pb.no_args_func_)
# resume_pub = ProtoPublisher("resume",robot_pb.no_args_func_)
# tobogan_pub = ProtoPublisher("set_toboggan",robot_pb.SetState)
# claw_pub= ProtoPublisher("set_pince",robot_pb.SetState)

# debug_pub =StringPublisher("debug_msg")
# proximitySub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)

# def bouge(x,y, theta):
#     pos_pub.send(robot_pb.Position(x=x,y=y,theta=theta))

# def slow():    
#     print("slowing")
#     slow_pub.send(robot_pb.no_args_func_(nothing = 1))

# def stop():
#     print("stoped")
#     stop_pub.send(robot_pb.no_args_func_(nothing = 1))

# def resume():
#     print("resumed")
#     resume_pub.send(robot_pb.no_args_func_(nothing = 1))

# def tobogan(cmd):
#     print("tobogged")
#     tobogan_pub.send(robot_pb.SetState(cerise_drop = cmd))
    
# def claw(state):
#     print("clawed")
#     claw_pub.send(robot_pb.SetState(claw_state = state))

# def onProximityStatus (self,topic_name, msg, timestamp):
#     debug_pub.send(str(msg.status))
#     if msg.status == lidar_pb.ProximityStatus.OK:
#         self.resume()
#     if msg.status == lidar_pb.ProximityStatus.WARNING:
#         self.slow()
#     if msg.status == lidar_pb.ProximityStatus.STOP:
#         self.stop()
#proximitySub.set_callback(onProximityStatus)

mama = Robot()

def end():
    ecal_core.finalize()
