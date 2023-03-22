import sys
import time
import numpy as np

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher, StringPublisher

from loca_lidar.ObstacleCalc import ObstacleCalc
import loca_lidar.CloudPoints as cp
import loca_lidar.PatternFinder as pf
import loca_lidar.lidar_data_pb2 as lidar_pb
import loca_lidar.robot_state_pb2 as robot_pb
from loca_lidar.PointsDataStruct import PolarPts
import loca_lidar.config as config


blue_beacons = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.known_points_in_mm))

ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")

sub_angle = ProtoSubscriber("robot_moving_angle", robot_pb.Travel)
sub_robot_pos = ProtoSubscriber("robot_pos", robot_pb.Position)
sub_lidar = ProtoSubscriber("lidar_data", lidar_pb.Lidar)

pub_stop_cons = StringPublisher("stop_cons") # pub_stop_cons = ProtoPublisher("stop_cons", lidar_pb.Action)
pub_filtered_pts = ProtoPublisher("lidar_filtered", lidar_pb.Lidar)
pub_amalgames = ProtoPublisher("amalgames", lidar_pb.Lidar)
pub_beacons = ProtoPublisher("beacons", lidar_pb.Lidar) # Only up to 5 points are sent, the index correspond to the fixed_point

last_known_moving_angle = 0 #angle in degrees from where the robot is moving 
last_known_lidar = (0, 0, 0) #x, y, theta (meters, degrees)
robot_pose = (0.0, 0.0, 0.0) #x, y, theta (meters, degrees)
OBSTACLE_CALC = ObstacleCalc(
    config.lidar_x_offset, config.lidar_y_offset, config.lidar_theta_offset)

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

def on_robot_pos(topic_name, pos_msg, time):
    global robot_pose
    robot_pose = (pos_msg.x, pos_msg.y, pos_msg.theta)


def on_lidar_scan(topic_name, proto_msg, time):
    global last_known_moving_angle, robot_pose, last_known_lidar

    # Obstacle avoidance
    lidar_scan =  np.rec.fromarrays([proto_msg.distances, proto_msg.angles], dtype=PolarPts)
    basic_filtered_scan = cp.basic_filter_pts(lidar_scan)
    obstacle_consigne = cp.obstacle_in_cone(basic_filtered_scan, last_known_moving_angle)
    send_stop_cons(-1, obstacle_consigne) # TODO : implement closest distance (currently sending -1)

    # advanced filtering
    x,y,theta = last_known_lidar[0], last_known_lidar[1], last_known_lidar[2]
    pos_filtered_scan = cp.position_filter_pts(basic_filtered_scan, x, y, theta)
    # TODO : position filter is unimplemented | it returns everything

    # Obstacle Calculation
    obs = OBSTACLE_CALC.calc_obstacles_wrt_table(robot_pose, pos_filtered_scan) #type: ignore
    print(obs)
    # Sending filtered & amalgames data for visualization
    send_lidar(pub_filtered_pts, pos_filtered_scan['distance'], pos_filtered_scan['angle']) # Display filtered data for debugging purposes
    amalgames = cp.amalgames_from_cloud(pos_filtered_scan)
    send_lidar(pub_amalgames, amalgames['center_polar']['distance'], amalgames['center_polar']['angle']) # Display filtered data for debugging purposes

    #position calculation
    
   
if __name__ == "__main__":

    sub_angle.set_callback(on_moving_angle)
    sub_lidar.set_callback(on_lidar_scan)

    while ecal_core.ok():
        time.sleep(0.5)

    ecal_core.finalize()
