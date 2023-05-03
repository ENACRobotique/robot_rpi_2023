import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import comm


class EcalRadio(comm.Radio):
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
        self.match_report_pub = ProtoPublisher('match_start',robot_pb.Match)
        self.action_report_pub = ProtoPublisher('action',robot_pb.Action)
        self.message_pub = StringPublisher('debug_msg')


        # High level to Low level message transmition 
        # Create the subscribers & set the callbacks
        self.set_position_sub = ProtoSubscriber("set_position", robot_pb.Position)
        self.set_position_sub.set_callback(self.on_set_position)
        

        self.reset_pos_sub = ProtoSubscriber("reset",robot_pb.Position)
        self.reset_pos_sub.set_callback(self.on_reset_pos)
        
        self.slow_sub = ProtoSubscriber("slow",robot_pb.no_args_func_)
        self.slow_sub.set_callback(self.on_slow)
        
        self.stop_sub = ProtoSubscriber("stop",robot_pb.no_args_func_)
        self.stop_sub.set_callback(self.on_stop)
        
        self.resume_sub = ProtoSubscriber("resume",robot_pb.no_args_func_)
        self.resume_sub.set_callback(self.on_resume)

        self.pince_sub = ProtoSubscriber("set_pince", robot_pb.SetState)
        self.pince_sub.set_callback(self.on_claw_command)

        self.trieuse_store_sub = ProtoSubscriber("store_disk", robot_pb.SetState)
        self.trieuse_store_sub.set_callback(self.on_store_disk)

        self.trieuse_drop_sub = ProtoSubscriber("drop_disk", robot_pb.SetState)
        self.trieuse_drop_sub.set_callback(self.on_drop_disk)
        
        self.toboggan_sub = ProtoSubscriber("set_toboggan", robot_pb.SetState)
        self.toboggan_sub.set_callback(self.on_toboggan)
        
        self.costume_sub = ProtoSubscriber("costume",robot_pb.no_args_func_)
        self.costume_sub.set_callback(self.sendCostumeSignal)
 
        self.score_sub = ProtoSubscriber("set_score", robot_pb.Match)
        self.score_sub.set_callback(self.on_score)

        # self.end_match_sub = ProtoSubscriber("end_match", robot_pb.Match)
        # self.end_match_sub.set_callback(self.on_end_match)
        
        self.proximity_sub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        #self.proximity_sub.set_callback()


        

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
        print("position",position)
        
    def on_claw_command(self,topic_name,setstate, time):
        self.sendClawSignal(setstate.claw_state)
        self.handle_message("ClAW MOVED")
        print("claw",setstate)
        
    def on_store_disk(self,topic_name,setstate, time):
        self.sendStoreDiscsInsideSignal(setstate.plate_position, setstate.plate_number)
        print("store",setstate.plate_position,setstate.plate_number)

    def on_drop_disk(self,topic_name,setstate, time):
        self.sendPicDiscFromStorage(setstate.plate_position, setstate.plate_number)
        print("drop",setstate.plate_position,setstate.plate_number)
        
    def on_toboggan(self,topic_name,setstate, time):
        self.sendTobogganSignal(setstate.cerise_drop)
        self.handle_message("TOBOGAN MOVED")
        print("cerise",setstate.cerise_drop)
        
    def on_score(self,topic_name,match, time):
        self.sendPointDisplay(match.score)
        print("score",match.score)
    
    def on_reset_pos(self,topic_name,position, time):
        self.resetPosition(position.x,position.y,position.theta)
        print("reset pos",position)
        
    def on_slow(self,topic_name,nothing,time):
        self.sendSlowDownSignal()
    
    def on_resume(self,topic_name,nothing,time):
        self.sendResumeSignal()
    
    def on_stop(self,topic_name,nothing,time):
        self.sendStopSignal()

    
        
    # def on_end_match(self,topic_name,match, time):
    #     self.(match.)
    #     print("",)
        


if __name__ == "__main__":
    radio = EcalRadio()
    test_pub = ProtoPublisher('set_position',robot_pb.Position)
    while ecal_core.ok():
        time.sleep(0.1)

    ecal_core.finalize()


    