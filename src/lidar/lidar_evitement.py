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


last_status = 'stop'
lidar_sub = None
proximity_status_pub = None


def compute_distances(topic_name, msg, time):
    global last_status
    global proximity_status_pub

    pts = np.array(msg.distances)
    pts_filter = pts[(pts>MIN_RADIUS)] # Removing points closer than MIN_RADIUS

    # Check if there are still point left before applying reduction operation
    if len(pts_filter) > 0:
        closest = np.min(pts_filter)

        # Check cylinder location and send messages to robot
        msg = lidar_data.Proximity()
        msg.closest_distance = closest

        if closest <= STOP_RADIUS:
            current_status = 'stop'
            msg.status = lidar_data.ProximityStatus.STOP
        elif closest <= SLOW_RADIUS:
            current_status = 'slow'
            msg.status = lidar_data.ProximityStatus.WARNING
        else:
            current_status = 'ok'
            msg.status = lidar_data.ProximityStatus.OK

        if current_status == last_status:
            proximity_status_pub.send(msg)
            print(current_status)
        
        last_status = current_status
    
if __name__ == '__main__':
    print('Starting ultimate-fallback collision avoidance')
    ecal_core.initialize(sys.argv, "lidar_evitement")

    proximity_status_pub = ProtoPublisher("proximity_status", lidar_data.Proximity)
    lidar_sub = ProtoSubscriber("lidar_data", lidar_data.Lidar)
    lidar_sub.set_callback(compute_distances)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()
