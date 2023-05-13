import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import time, os, sys
import numpy as np
from math import radians
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.lidar_data_pb2 as lidar_data
import generated.robot_state_pb2 as robot_data

MIN_RADIUS = 20.0e-2     # Filter points on robot
TOLERANCE = 5.0e-2
MAX_RADIUS = 1.0
SLOW_RADIUS = 80.0e-2   # Warning radius: start sending slow messages
STOP_RADIUS = 40.0e-2   # Panic radius: immediately send stop messages
LIDAR_OFFSET = np.radians(45.12) + np.pi

last_status = 'stop'
last_position = [0.0, 0.0, 0.0] # (x, y, theta)
lidar_data_sub = None
odom_position_pub = None
proximity_status_pub = None
lidar_filtered_pub = None

def send_lidar_scan(pub, distances, angles):
    lidar_msg = lidar_data.Lidar()
    lidar_msg.angle_increment = float(-1.0) # prevent empty message when sending empty lidar scan (eg no obstacle found)
    lidar_msg.angles.extend(angles)
    lidar_msg.distances.extend(distances)
    pub.send(lidar_msg, ecal_core.getmicroseconds()[1])


def compute_distances(topic_name, msg, time):
    global last_status
    global last_position
    global proximity_status_pub
    global lidar_filtered_pub

    distances = np.array(msg.distances)
    angles = np.radians(np.array(msg.angles))

    # Filter points that are too far or too close to be useful -> BASIC filter
    indexes = np.logical_and(distances > MIN_RADIUS, distances <= MAX_RADIUS).nonzero()
    basic_distances_filter = distances[indexes]
    raw_basic_angles_filter = angles[indexes] # angles without lidar offset
    basic_angles_filter = raw_basic_angles_filter - LIDAR_OFFSET

    # Coordinate transform to remove points outside the table
    x = basic_distances_filter * np.cos(basic_angles_filter + last_position[2]) + last_position[0]
    y = basic_distances_filter * np.sin(basic_angles_filter + last_position[2]) + last_position[1]
    cond_x = np.logical_and(x > TOLERANCE, x < (3.0 - TOLERANCE))
    cond_y = np.logical_and(y > TOLERANCE, y < (2.0 - TOLERANCE))
    indexes = np.logical_and(cond_x, cond_y).nonzero()
    
    distances_filter = basic_distances_filter[indexes]

    # send lidar points that are detected on table (for visualization debug purposes)
    send_lidar_scan(lidar_filtered_pub, 
                    basic_distances_filter[indexes], 
                    np.degrees(raw_basic_angles_filter[indexes])) # convert to positive deg angles

    msg = lidar_data.Proximity()
    if len(distances_filter) == 0:
        current_status = 'ok'
        msg.status = lidar_data.ProximityStatus.OK

    else:
        closest = np.min(distances_filter)

        # Check cylinder location and send messages to robot
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


    """
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
    """
    

def get_position(topic_name, msg, time):
    global last_position

    last_position[0] = msg.x
    last_position[1] = msg.y
    last_position[2] = msg.theta

    
if __name__ == '__main__':
    print('Starting ultimate-fallback collision avoidance')
    ecal_core.initialize(sys.argv, "lidar_evitement")

    proximity_status_pub = ProtoPublisher("proximity_status", lidar_data.Proximity)
    lidar_data_sub = ProtoSubscriber("lidar_data", lidar_data.Lidar)
    lidar_filtered_pub = ProtoPublisher("lidar_filtered", lidar_data.Lidar)
    lidar_data_sub.set_callback(compute_distances)
    odom_position_sub = ProtoSubscriber('odom_pos', robot_data.Position)
    odom_position_sub.set_callback(get_position)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()
