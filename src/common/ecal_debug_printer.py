import ecal.core.core as ecal_core
import time
from ecal.core.subscriber import ProtoSubscriber
from ecal.pb.monitoring_pb2 import Logging

# convert micro seconds timestamp of int type to 24 hour clock format
#Display time in 24 hour format in UTC (due to unix timestamp)
def us_to_hour(time):
    hour = time // 3600000000 % 24
    minute = (time % 3600000000) // 60000000
    second = (time % 60000000) // 1000000
    return "{:02d}:{:02d}:{:02d}".format(hour, minute, second)


def on_log(topic, msg, time):
    for msg in msg.logs:
        msg_time = us_to_hour(msg.time)
        ### TODO : print automatically program name, but currently it print the python interpreter (which is the same for every script...) 
        # #split string by "\":
        ### full_program_path = msg.uname.replace("\\", "/") # support windows path
        ### program_name = full_program_path.split("/")[-1] # split the path and get only program name
        print(msg_time, msg.level, msg.content)
        #pname, content, level, pid, pname, uname

if __name__ == "__main__":

    ecal_core.mon_initialize()
    ecal_core.initialize([], "ecal_printer")

    ecal_core.enable_loopback(1)

    #TODO if not already ropic ecal_log : 
    ecal_core.mon_publogging(1, "ecal_log")
    sub = ProtoSubscriber("ecal_log", Logging)


    time.sleep(1.0)

    while ecal_core.ok():
        time.sleep(0.01)
        on_log(*sub.receive(0))