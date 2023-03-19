import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.service import Server
import time, sys
import numpy as np

import loca_lidar.robot_state_pb2 as robot_pb
import loca_lidar.lidar_data_pb2 as lidar_pb
import loca_lidar.config as config
from loca_lidar.PatternFinder import polar_lidar_to_cartesian
from loca_lidar.CloudPoints import get_squared_dist_cartesian

"""TODO
    - pour la gestion des déplacements sur la carte:
    - la stratégie(module nav) demande: qqun à cet endroit là?
    - le lidar répond: {"oui"; "non"}
"""
# detected from lidar
last_lidar_dist = ((0))
last_lidar_angles = ((0))
# create a transformation matrix from Lidar Frame to Table Frame
t_lidar_wrt_table = None

def _is_in_circle(pt, circle_center = (0, 0)):
    if np.sqrt(get_squared_dist_cartesian(pt, circle_center)) <= config.obstacle_radius:
        return True 
    return False

def _check_obstacle(circle_coor=(1.0, 180)): #(r, theta)
    global t_lidar_wrt_table, last_lidar_dist, last_lidar_angles
    if t_lidar_wrt_table == None or last_lidar_dist == ((0)):
        raise Exception("lidar position and/or lidar cloud points are not found - can't process nav obstacle request")

    # check each point from lidar cloud if it's within a circle centered on the given x,y coordinate
    for i, dist in enumerate(last_lidar_dist): 
        pt_wrt_lidar = polar_lidar_to_cartesian(dist, last_lidar_angles[i])  # coordinate of the point with regards to lidar frame

        # create a homogeneous coordinate vector for this lidar point in lidar frame
        v_C_B = np.array([pt_wrt_lidar[0], pt_wrt_lidar[1], 1])
        # transform the homogeneous coordinate vector from Lidar Frame to Table Frame
        v_C_A = np.dot(t_lidar_wrt_table, v_C_B)
        pt_wrt_table = (v_C_A[0], v_C_A[1])
        if _is_in_circle(pt_wrt_table, circle_coor):
            return True
    return False

def on_nav_request(method_name, req_type, resp_type, request):
    # https://stackoverflow.com/questions/8494514/converting-string-to-tuple
    is_free = _check_obstacle(tuple(eval(request.decode())))
    return 0, bytes('1', "ascii") if is_free else 0, bytes('0', "ascii")
    

def on_lidar(topic_name, proto_msg, time): 
    global last_lidar_angles, last_lidar_dist
    last_lidar_dist = proto_msg.distances
    last_lidar_angles = proto_msg.angles

def on_robot_position(topic_name, proto_msg, time): 
    global t_lidar_wrt_table
    t_lidar_wrt_table = np.array([
        [np.cos(proto_msg.theta), -np.sin(proto_msg.theta), proto_msg.x],
        [np.sin(proto_msg.theta), np.cos(proto_msg.theta), proto_msg.y],
        [0, 0, 1]
    ])

if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "obstacle_request_server")

    sub_filtered_pts = ProtoSubscriber("lidar_filtered", lidar_pb.Lidar)
    sub_lidar_pos = ProtoSubscriber("lidar_pos", robot_pb.Position)

    request_serv =  Server('check_position')
    request_serv.add_method_callback('is_free', 
        req_type=str(robot_pb.Position),
        resp_type=str(bool),
        callback = on_nav_request
        )


    while ecal_core.ok():
        time.sleep(0.5)

    ecal_core.finalize()



