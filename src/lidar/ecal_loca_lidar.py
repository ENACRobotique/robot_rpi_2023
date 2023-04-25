import logging
import sys, os
import time
from typing import Tuple, Union
import numpy as np

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher, StringPublisher

from loca_lidar.ObstacleCalc import ObstacleCalc
import loca_lidar.CloudPoints as cp
import loca_lidar.PatternFinder as pf
from loca_lidar.PointsDataStruct import PolarPts
import loca_lidar.config as config

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.lidar_data_pb2 as lidar_pb
import generated.robot_state_pb2 as robot_pb


BLUE_BEACONS = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.known_points_in_mm))

ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")

sub_angle = ProtoSubscriber("odom_speed", robot_pb.Position)
sub_odom_pos = ProtoSubscriber("odom_pos", robot_pb.Position)
sub_lidar = ProtoSubscriber("lidar_data", lidar_pb.Lidar)

pub_stop_cons = ProtoPublisher("proximity_status", lidar_pb.Proximity) # pub_stop_cons = ProtoPublisher("stop_cons", lidar_pb.Action)
pub_filtered_pts = ProtoPublisher("lidar_filtered", lidar_pb.Lidar)
pub_amalgames = ProtoPublisher("amalgames", lidar_pb.Lidar)
pub_beacons = StringPublisher("beacons") # Only up to 5 points are sent, the index correspond to the fixed_point
pub_lidar_pos = ProtoPublisher("lidar_pos", robot_pb.Position)
pub_obstacles = ProtoPublisher("obstacles_wrt_table", lidar_pb.Obstacles)

last_known_speed = (0.0, 0.0, 0.0) #angle in degrees from where the robot is moving 
last_known_lidar = (0, 0, 0) #x, y, theta (meters, degrees)
robot_pose = (0.0, 0.0, 0.0) #x, y, theta (meters, degrees)
OBSTACLE_CALC = ObstacleCalc(
    config.lidar_x_offset, config.lidar_y_offset, config.lidar_theta_offset)

BLUE_BEACONS = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.known_points_in_mm), True)
FINDER = pf.LinkFinder(BLUE_BEACONS, 0.06, 1.5)

def send_obstacles_wrt_table(obstacles: list[list[Union[float, float]]]):
    msg = lidar_pb.Obstacles()
    x, y = [], []
    for obstacle in obstacles:
        x.append(obstacle[0])
        y.append(obstacle[1])
    msg.x.extend(x)
    msg.y.extend(y)
    pub_obstacles.send(msg, ecal_core.getmicroseconds()[1])
    
def send_stop_cons(closest_distance: float, action: int):
    msg = lidar_pb.Proximity()
    if action == 0:
        msg.status = lidar_pb.ProximityStatus.OK
    elif action == 1:
        msg.status = lidar_pb.ProximityStatus.WARNING
    elif action == 2:
        msg.status = lidar_pb.ProximityStatus.STOP
    else:
        raise ValueError("ecal_loca_lidar - send_stop_cons - Invalid action value")
    pub_stop_cons.send(msg, ecal_core.getmicroseconds()[1])

def send_lidar_scan(pub, distances, angles):
    lidar_msg = lidar_pb.Lidar()
    lidar_msg.angle_increment = float(-1.0) # prevent empty message when sending empty lidar scan (eg no obstacle found)
    lidar_msg.angles.extend(angles)
    lidar_msg.distances.extend(distances)
    pub.send(lidar_msg, ecal_core.getmicroseconds()[1])

def send_lidar_pos(x, y, theta):
    pos_msg = robot_pb.Position()
    pos_msg.x = float(x)
    pos_msg.y = float(y)
    pos_msg.theta = float(theta)
    pub_lidar_pos.send(pos_msg, ecal_core.getmicroseconds()[1])


def on_robot_speed(topic_name, travel_msg, time):
    global last_known_speed
    last_known_speed = (travel_msg.x, travel_msg.y, travel_msg.theta)

def on_robot_pos(topic_name, pos_msg, time):
    global robot_pose
    robot_pose = (pos_msg.x, pos_msg.y, pos_msg.theta)


def on_lidar_scan(topic_name, proto_msg, time):
    global last_known_speed, robot_pose, last_known_lidar

    if robot_pose == (0.0, 0.0, 0.0):
        logging.warning("Robot pose not received yet - invalid obstacle avoidance")

    t = ecal_core.getmicroseconds()[1]
    # Filter lidar_scan
    lidar_scan =  np.rec.fromarrays([proto_msg.distances, proto_msg.angles], dtype=PolarPts)
    basic_filtered_scan = cp.basic_filter_pts(lidar_scan)
    # TODO : position filter is unimplemented | it returns everything
    pos_filtered_scan = cp.position_filter_pts(basic_filtered_scan, 
                        robot_pose[0], robot_pose[1], robot_pose[2])
    obs = OBSTACLE_CALC.calc_obstacles_wrt_table(robot_pose, pos_filtered_scan) #type: ignore
    mask = OBSTACLE_CALC.mask_filter_obs(obs) # truth list if on table 
    # TODO : rework position_filter_pts to return a mask instead of giving the work to OBSTACLE_CALC
    pos_filtered_scan = pos_filtered_scan[mask]

    #obstacle avoidance
    filtered_obs = [obs[i] for i in range(len(obs)) if mask[i]]

    obstacle_consigne = cp.obstacle_in_path(robot_pose, filtered_obs, last_known_speed)
    send_stop_cons(-1, obstacle_consigne) # TODO : implement closest distance (currently sending -1)
    send_obstacles_wrt_table(filtered_obs)

    # Obstacle Calculation
    # Sending filtered & amalgames data for visualization
    send_lidar_scan(pub_filtered_pts, pos_filtered_scan['distance'], pos_filtered_scan['angle']) # Display filtered data for debugging purposes
    # amalgames don't use position filter because it removes points outside the table
    amalgames = cp.amalgames_from_cloud(basic_filtered_scan)
    amalgames = cp.filter_amalgame_size(amalgames)
    send_lidar_scan(pub_amalgames, amalgames['center_polar']['distance'], amalgames['center_polar']['angle']) # Display filtered data for debugging purposes

    #position calculation
    lidar2table = {}
    #TODO : remove empty amalgames['centerpolar]
    lidar_pose = calculate_lidar_pose(amalgames['center_polar'], robot_pose, lidar2table)
    if lidar_pose != (0, 0, 0):
        pub_beacons.send(str(lidar2table))
        send_lidar_pos(*lidar_pose)

    t2 = ecal_core.getmicroseconds()[1] - t
    print("processing duration total in ms : ",t2)

def calculate_lidar_pose(amalgame_scan, robot_pose = (0.0, 0.0, 0.0), corr_out = {}) -> Tuple[float, float, float]:
    """_summary_

    Args:
        lidar_scan (_type_): amalgame from lidar scan ((distance, angle), ...)
        corr_out (dict, optional): Correspondance lidar2table dict. Modify this dictionary. Can be used for visualization purpose. Defaults to {}.

    Returns:
        Tuple[float, float, float]: _description_
    """
    #Find correspondances between lidar and table
    amalgame_1 = pf.GroupAmalgame(amalgame_scan, False)
    lidar2table_set = FINDER.find_pattern(amalgame_1)

    if lidar2table_set == None:
        logging.warning("No correspondance found between lidar and table")
        return (0, 0, 0)

    poses = []
    best_pose = ()
    for i, corr in enumerate(lidar2table_set):
        lidar_pos = pf.lidar_pos_wrt_table(
            corr, amalgame_1.points, BLUE_BEACONS.points)
        lidar_angle = pf.lidar_angle_wrt_table(
            lidar_pos, corr, amalgame_1.points, BLUE_BEACONS.points)
        if len(lidar2table_set) == 1: # trivial case
            best_pose = (lidar_pos[0], lidar_pos[1], lidar_angle)
            corr_out |= corr #fusion the two dicts, to make sure that outside the function the dict is not empty
            break
        poses.append((lidar_pos[0], lidar_pos[1], lidar_angle))

    if poses != []: # if multiple poses found, select one of them      
        print("lidar2table_set", lidar2table_set)
        #eliminate poses outside the tables
        poses = [pose for pose in poses if 
                 pose[0] > config.table_x_min and pose[0] < config.table_x_max 
                 and pose[1] > config.table_y_min and pose[1] < config.table_y_max]
        # take the pose closest to odometry pose
        closest_pt_index = poses.index(min(poses, key=lambda x: cp.get_squared_dist_cartesian(robot_pose[:2], x)))
        best_pose = poses[closest_pt_index]
        corr_out |= list(lidar2table_set)[closest_pt_index] #fusion the two dicts, to make sure that outside the function the dict is not empty (set is not subscriptable so convert to list)
        print(poses)

    return best_pose
   
if __name__ == "__main__":

    sub_angle.set_callback(on_robot_speed)
    sub_lidar.set_callback(on_lidar_scan)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()
