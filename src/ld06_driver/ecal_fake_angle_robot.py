import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher

import robot_state_pb2 as robot_state
import time, sys

ecal_core.initialize(sys.argv, "eCAL_LD06_driver")

x = 0.1
pub = ProtoPublisher("robot_moving_angle", robot_state.Travel)
def publish_reading():
    #once the program finished to read a full circle reading from lidar, publlish it to eCAL with protobuf format
    travel_msg = robot_state.Travel()
    travel_msg.theta = x
    pub.send(travel_msg, ecal_core.getmicroseconds()[1])

if __name__ == "__main__":
    # print("cone is moving 10° by 10° every second")
    while True: 
        time.sleep(1)
        # x += 10
        # x %= 360
        publish_reading()