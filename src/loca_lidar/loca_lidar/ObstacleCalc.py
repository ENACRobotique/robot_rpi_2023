# Manage Obstacles position wrt Table - can be used for planning by navigation (check if a place is free)

from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from typing import Tuple, List, Union


@dataclass
class ObstacleCalc():
    # last_lidar_dist: tuple
    x_offset: float
    y_offset: float
    theta_offset: float # offset of the lidar relative to the robot center (meters, degrees)

    def __post_init__(self):
        if self.x_offset is None: self.x_offset = 0
        if self.y_offset is None: self.y_offset = 0
        if self.theta_offset is None: self.theta_offset = 0


    def calc_obstacles_wrt_table(self, 
        robot_wrt_table: tuple, lidar_pts: Tuple[Tuple[float, float], ...]) -> List[list[Union[float, float]]]: 
        # Calculate and returns the cartesian coordinates of the obstacles 
        # given the lidar polar points and the lidar position on the table
        # !! Filter the obstacles outside the table
        # !! theta angle is parallel to the y axis currently
        obstacles = []
        lidar_wrt_robot = self._lidar_wrt_robot(robot_wrt_table)
        for pt in lidar_pts: # May be possible to vectorize with numpy to avoid for loop
            # From Lidar Frame to Robot Frame
            pt_wrt_lidar = self.polar_lidar_to_cartesian(pt)
            # create a homogeneous coordinate vector for this lidar point in lidar frame
            v_C_B = np.array([pt_wrt_lidar[0], pt_wrt_lidar[1], 1])
            # transform the homogeneous coordinate vector from Lidar Frame to Robot Frame
            v_C_A = np.dot(self.t_lidar_to_robot, v_C_B)
            pt_wrt_robot = (v_C_A[0], v_C_A[1])

            #From Robot Frame to Table Frame
            v_C_B = np.array([pt_wrt_robot[0], pt_wrt_robot[1], 1])
            v_C_A = np.dot(self.t_robot_to_table(robot_wrt_table), v_C_B)
            pt_wrt_table = (v_C_A[0], v_C_A[1])
            obstacles.append(pt_wrt_table)

        return self._filter_obstacles(obstacles)

    @staticmethod
    def polar_lidar_to_cartesian(polar_coord: tuple[float, float]):
    #input : (r, theta) 
    # r = distance ; theta = angle in DEGREES
        r = polar_coord[0]
        theta = np.radians(polar_coord[1]) + np.pi/2 #adding offset
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return np.array([x, y])
    
    @property
    def t_lidar_to_robot(self): #transform matrix from lidar to robot
        return np.array([
        [np.cos(self.theta_offset), -np.sin(self.theta_offset), self.x_offset],
        [np.sin(self.theta_offset), np.cos(self.theta_offset), self.y_offset],
        [0, 0, 1]
    ])

    @staticmethod
    @lru_cache(10)
    def t_robot_to_table(robot_pose: Tuple[float, float, float]):
        # transform matrix from robot to table
        rad_theta = np.deg2rad(robot_pose[2])
        return np.array([
        [np.cos(rad_theta), -np.sin(rad_theta), robot_pose[0]],
        [np.sin(rad_theta), np.cos(rad_theta), robot_pose[1]],
        [0, 0, 1]
    ])
    

    def _lidar_wrt_robot(self, robot_wrt_table):
        # create a homogeneous coordinate vector for this lidar point in lidar frame
        v_C_B = np.array([robot_wrt_table[0], robot_wrt_table[1], 1])
        # transform the homogeneous coordinate vector from Lidar Frame to Table Frame
        v_C_A = np.dot(self.t_lidar_to_robot, v_C_B)
        return (v_C_A[0], v_C_A[1]) 

    @staticmethod
    def _filter_obstacles(obstacles: list[tuple[float, float]]):
        # removes obstacles outside the table
        filtered_obstacles = []
        for obs in obstacles:
            if not (obs[0] < 0 or obs[0] > 2.0 or obs[1] < 0 or obs[1] > 3.0):
                filtered_obstacles.append(obs)
        return filtered_obstacles

    



