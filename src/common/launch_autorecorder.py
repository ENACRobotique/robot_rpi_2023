import ecal.core.core as ecal_core
import time, os
from ecal.core.subscriber import ProtoSubscriber
import generated.robot_state_pb2 as robot_pb

IS_RECORDING = False
#CAUTION UNTESTED FILE !!

def on_match_start(topic_name, msg, time):
    global IS_RECORDING
    if IS_RECORDING:
        return # already recording
    
    # record for 110s after receiving match started
    if msg.tirette == robot_pb.IHM.TIRETTE_OUT :
        IS_RECORDING = True
        os.system('ecal_rec -r 110 --activate')

if __name__ == "__main__":
    ecal_core.initialize([], "autorecorder")

    ProtoSubscriber("ihm",robot_pb.IHM).set_callback(on_match_start)

    time.sleep(1.0)

    while ecal_core.ok():
        time.sleep(0.1)
    
    ecal_core.finalize()



