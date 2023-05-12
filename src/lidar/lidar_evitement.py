import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import time, os, sys
import numpy as np
from math import radians
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.lidar_data_pb2 as lidar_data

MIN_RADIUS = 20.0e-2     # Filter points on robot
SLOW_RADIUS = 60.0e-2   # Warning radius: start sending slow messages
STOP_RADIUS = 40.0e-2   # Panic radius: immediately send stop messages

print('Starting ultimate-fallback collision avoidance')
ecal_core.initialize(sys.argv, "lidar_evitement")

lidar_sub = ProtoSubscriber("lidar_data", lidar_data.Lidar)
proximity_status_pub = ProtoPublisher("proximity_status", lidar_data.Proximity)


def compute_distances(topic_name, msg, time):
    pts = np.array(msg.distances)
    pts_filter = pts[(pts>MIN_RADIUS)] # Removing points closer than MIN_RADIUS

    # Check if there are still point left before applying reduction operation
    if len(pts_filter) > 0:
        closest = np.min(pts_filter)
    else:
        closest = STOP_RADIUS

    # Check cylinder location and send messages to robot
    msg = lidar_data.Proximity()
    msg.closest_distance = closest
    if closest <= STOP_RADIUS:
        print('STOP')
        msg.status = lidar_data.ProximityStatus.STOP
    elif closest <= SLOW_RADIUS:
        print('SLOW')
        msg.status = lidar_data.ProximityStatus.WARNING
    else:
        print('OK')
        msg.status = lidar_data.ProximityStatus.OK
    proximity_status_pub.send(msg)
    

lidar_sub.set_callback(compute_distances)

while ecal_core.ok():
    time.sleep(0.01)

ecal_core.finalize()

