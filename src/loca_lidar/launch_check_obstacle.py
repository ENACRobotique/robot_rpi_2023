import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.service import Server
import time, sys

import loca_lidar.robot_state_pb2 as robot_pb
# sub_filtered_pts = ProtoSubscriber("lidar_filtered", lidar_pb.Lidar)

def on_nav_request():
    pass

ecal_core.initialize(sys.argv, "obstacle_request_server")

request_serv =  Server('is_free_position')
request_serv.add_method_callback('on_nav_request', 
    req_type=str(robot_pb.Position),
    resp_type=str(bool),
    callback = on_nav_request
    )


while ecal_core.ok():
    time.sleep(0.5)

ecal_core.finalize()



