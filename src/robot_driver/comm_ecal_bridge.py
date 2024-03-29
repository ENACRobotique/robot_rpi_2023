import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

import sys

if len(sys.argv) > 1 and sys.argv[1] == "sim":
    from simu import Radio
else:
    from comm import Radio


class EcalRadio(Radio):
    def __init__(self):
        super().__init__()

        ecal_core.initialize(sys.argv, "serial->ecal bridge")
        self.init_ecal_sig()
        time.sleep(1.0) # time needed to initialize ecal
        print("ecal initialized too !")
        self.startListening()

    def init_ecal_sig(self):
        """Initialize eCAL"""
        
        # Low level to High level message transmition
        self.odom_position_pub = ProtoPublisher('odom_pos', robot_pb.Position)
        self.odom_speed_pub = ProtoPublisher('odom_speed',robot_pb.Speed)
        self.action_report_pub = ProtoPublisher('action',robot_pb.Action)
        self.message_pub = StringPublisher('debug_msg')
        self.cake_presence_pub = ProtoPublisher("cake_presence",robot_pb.Action)
        self.ihm_pub = ProtoPublisher("ihm",robot_pb.IHM)

        # High level to Low level message transmition 
        # Create the subscribers & set the callbacks
        self.set_position_sub = ProtoSubscriber("set_position", robot_pb.Position)
        self.set_position_sub.set_callback(self.on_set_position)

        self.reset_pos_sub = ProtoSubscriber("reset",robot_pb.Position)
        self.reset_pos_sub.set_callback(self.on_reset_pos)

        self.pince_sub = ProtoSubscriber("set_pince", robot_pb.SetState)
        self.pince_sub.set_callback(self.on_claw_command)
        
        self.toboggan_sub = ProtoSubscriber("set_toboggan", robot_pb.SetState)
        self.toboggan_sub.set_callback(self.on_toboggan)
        
        self.costume_sub = ProtoSubscriber("costume",robot_pb.no_args_func_)
        self.costume_sub.set_callback(self.on_costume)
 
        self.score_sub = ProtoSubscriber("set_score", robot_pb.Match)
        self.score_sub.set_callback(self.on_score)

        self.proximity_sub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        self.proximity_sub.set_callback(self.on_proximity_status)

        time.sleep(1.0)
        self.odom_speed_pub.send(robot_pb.Speed(vx=0, vy=0, vtheta=0)) # send a first message to 'initialize' the topic (needed for lidar_fusion_smooth)
        
        # self.end_match_sub = ProtoSubscriber("end_match", robot_pb.Match)
        # self.end_match_sub.set_callback(self.on_end_match)
        

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
    
    def handle_cake_presence_report(self,is_cake):
        self.cake_presence_pub.send(robot_pb.Action(action=is_cake))
    
    def handle_IHM(self, tirette, color, posdep):
        self.ihm_pub.send(robot_pb.IHM(tirette=tirette, color=color, posdep=posdep))
    
    #Send cmd to robot
    def on_set_position(self, topic_name, position, time):
        self.setTargetPosition(position.x, position.y, position.theta)
        #print("position",position)
        
    def on_claw_command(self,topic_name,setstate, time):
        self.sendClawSignal(setstate.claw_state)
        #print("claw",setstate.claw_state)

        
    def on_toboggan(self,topic_name,setstate, time):
        self.sendTobogganSignal(setstate.cerise_drop)
        #print("cerise",setstate.cerise_drop)
        
    def on_score(self,topic_name,match, time):
        self.sendPointDisplay(match.score)
        #print("score",match.score)
    
    def on_reset_pos(self,topic_name,position, time):
        self.resetPosition(position.x,position.y,position.theta)
        #print("reset pos",position)
        
    def on_proximity_status(self,topic_name,msg,time):
        if msg.status == lidar_pb.ProximityStatus.OK:
            self.sendResumeSignal()
        elif msg.status == lidar_pb.ProximityStatus.WARNING:
            self.sendSlowDownSignal()
        elif msg.status == lidar_pb.ProximityStatus.STOP:
            self.sendStopSignal()

    def on_costume(self, topic, nothing, time):
        self.sendCostumeSignal()


    # def on_end_match(self,topic_name,match, time):
    #     self.(match.)
    #     print("",)


if __name__ == "__main__":
    radio = EcalRadio()
    while ecal_core.ok():
        time.sleep(0.1)

    ecal_core.finalize()


    
