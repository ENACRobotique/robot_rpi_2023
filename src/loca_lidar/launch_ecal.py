import sys
import time
import numpy as np

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher, StringPublisher

import loca_lidar.CloudPoints as cp
import loca_lidar.PatternFinder as pf
import loca_lidar.lidar_data_pb2 as lidar_pb
import loca_lidar.robot_state_pb2 as robot_pb
from loca_lidar.PointsDataStruct import PolarPts
import loca_lidar.config as config

"""TODO
    - pour la gestion des déplacements sur la carte:
    - la stratégie(module nav) demande: qqun à cet endroit là?
    - le lidar répond: {"oui"; "non"}
"""

blue_beacons = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.known_points_in_mm))

ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")

sub_angle = ProtoSubscriber("robot_moving_angle", robot_pb.Travel)
sub_lidar = ProtoSubscriber("lidar_data", lidar_pb.Lidar)

pub_stop_cons = StringPublisher("stop_cons") # pub_stop_cons = ProtoPublisher("stop_cons", lidar_pb.Action)
pub_filtered_pts = ProtoPublisher("lidar_filtered", lidar_pb.Lidar)
pub_amalgames = ProtoPublisher("amalgames", lidar_pb.Lidar)
pub_beacons = ProtoPublisher("beacons", lidar_pb.Lidar) # Only up to 5 points are sent, the index correspond to the fixed_point

last_known_moving_angle = 0 #angle in degrees from where the robot is moving 
last_known_lidar = (0, 0, 0) #x, y, theta (meters, degrees)

def send_stop_cons(closest_distance: float, action: int):
    # msg = lidar_pb.Proximity()
    # msg.action = lidar_pb.Action. ????
    pub_stop_cons.send(str(action))

def send_lidar(pub, distances, angles):
    lidar_msg = lidar_pb.Lidar()
    lidar_msg.angles.extend(angles)
    lidar_msg.distances.extend(distances)
    pub.send(lidar_msg, ecal_core.getmicroseconds()[1])


def on_moving_angle(topic_name, travel_msg, time):
    global last_known_moving_angle
    last_known_moving_angle = travel_msg.theta


def on_lidar_scan(topic_name, proto_msg, time):
    global last_known_moving_angle

    # Obstacle avoidance
    lidar_scan =  np.rec.fromarrays([proto_msg.distances, proto_msg.angles], dtype=PolarPts)
    basic_filtered_scan = cp.basic_filter_pts(lidar_scan)
    obstacle_consigne = cp.obstacle_in_cone(basic_filtered_scan, last_known_moving_angle)
    send_stop_cons(-1, obstacle_consigne) # TODO : implement closest distance (currently sending -1)

    # advanced filtering
    x,y,theta = last_known_lidar[0], last_known_lidar[1], last_known_lidar[2]
    pos_filtered_scan = cp.position_filter_pts(basic_filtered_scan, x, y, theta)
    # TODO : position filter is unimplemented | it returns everything
    send_lidar(pub_filtered_pts, pos_filtered_scan['distance'], pos_filtered_scan['angle']) # Display filtered data for debugging purposes
    amalgames = cp.amalgames_from_cloud(pos_filtered_scan)
    send_lidar(pub_amalgames, amalgames['center_polar']['distance'], amalgames['center_polar']['angle']) # Display filtered data for debugging purposes

   
if __name__ == "__main__":

    sub_angle.set_callback(on_moving_angle)
    sub_lidar.set_callback(on_lidar_scan)

    while ecal_core.ok():
        time.sleep(0.5)

    ecal_core.finalize()
