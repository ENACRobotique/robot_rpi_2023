import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import comm
from random import randint

class EcalRadio(comm.Radio):
    def __init__(self):
        super().__init__()

        ecal_core.initialize(sys.argv, "serial->ecal bridge")
        self.init_ecal_sub()
        time.sleep(1.0) # time needed to initialize ecal
        print("ecal initialized too !")
        self.startListening()

    def init_ecal_sub(self):
        """Initialize eCAL"""
        ecal_core.initialize(sys.argv, "serial->ecal bridge")
        
        # Low level to High level message transmition
        self.odom_position_pub = ProtoPublisher('odom_pos', robot_pb.Position)
        self.odom_speed_pub = ProtoPublisher('odom_speed',robot_pb.Speed)
        self.match_report_pub = ProtoPublisher('match_start',robot_pb.Match)
        self.action_report_pub = ProtoPublisher('action',robot_pb.Action)
        self.message_pub = StringPublisher('debug_msg')


        # High level to Low level message transmition 
        # Create the subscribers & set the callbacks
        self.set_position_sub = ProtoSubscriber("set_position", robot_pb.Position)
        self.set_position_sub.set_callback(self.on_set_position)
        
        self.proximity_sub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        #self.proximity_sub.set_callback()

        #self.pince_sub = ProtoSubscriber("set_pince", robot_pb.Claw)
        #self.pince_sub.set_callback(self.sendClawSignal)
#
        self.trieuse_put_out_sub = ProtoSubscriber("pick_disk", robot_pb.SetState)
        self.trieuse_put_out_sub.set_callback(self.sendStoreDiscsInsideSignal)

        # self._sub = ProtoSubscriber("drop_disk", robot_pb.SetState).set_callback(self.on_drop_disk)

        # self._sub = ProtoSubscriber("set_turbine", robot_pb.SetState).set_callback(self.on_turbine)

        # self._sub = ProtoSubscriber("set_toboggan", robot_pb.SetState).set_callback(self.on_toboggan)

        # self._sub = ProtoSubscriber("set_score", robot_pb.SetState).set_callback(self.on_score)

        # self._sub = ProtoSubscriber("end_match", robot_pb.Bool).set_callback(self.on_end_match)

    ####  Reception from robot
    def handle_pos_report(self, x, y, theta):
        self.odom_position_pub.send(robot_pb.Position(x=x, y=y, theta=theta))

    def handle_speed_report(self, vx, vy, vtheta):
        self.odom_speed_pub.send(robot_pb.Speed(vx=vx, vy=vy, vtheta=vtheta))

    def handle_match_report(self):
        self.match_report_pub.send(robot_pb.Match(status = "MATCH STARTED"))

    def handle_action_report(self,num):
        self.action_report_pub.send(robot_pb.Action(action=num))

    def handle_message(self,msg):
        self.message_pub.send(msg)

    def on_set_position(self, topic_name, position, time):
        self.setTargetPosition(position.x, position.y, position.theta)
        print(position)


if __name__ == "__main__":
    radio = EcalRadio()
    test_pub = ProtoPublisher('set_position',robot_pb.Position)
    while True:
        time.sleep(0.1)

    ecal_core.finalize()


    