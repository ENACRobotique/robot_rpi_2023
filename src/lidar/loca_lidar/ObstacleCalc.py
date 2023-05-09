# Manage Obstacles position wrt Table - can be used for planning by navigation (check if a place is free)

from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from typing import Tuple, List, Union
try:
    from loca_lidar.config import table_x_min, table_x_max, table_y_min, table_y_max, lidar_theta_offset
except ModuleNotFoundError:
    from config import table_x_min, table_x_max, table_y_min, table_y_max, lidar_theta_offset


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
    
    def calc_obstacles_wrt_table(self, robot_wrt_table: tuple, lidar_pts: Tuple[Tuple[float, float], ...]) -> List[List[Union[float, float]]]: 
        # Calculate and returns the cartesian coordinates of the obstacles 
        # given the lidar polar points and the lidar position on the table
        # !! Filter the obstacles outside the table
        # !! theta angle is parallel to the y axis currently
        obstacles = []
        rot_angle = (robot_wrt_table[2] + self.theta_offset)
        for pt in lidar_pts: # May be possible to vectorize with numpy to avoid for loop
            # From Lidar Frame to table frame directly
            pt_wrt_lidar = self.polar_lidar_to_cartesian(pt)
            rot_mat = np.array([[np.cos(rot_angle), -np.sin(rot_angle)], [np.sin(rot_angle), np.cos(rot_angle)]])
            pt_wrt_table = np.dot(rot_mat, pt_wrt_lidar) + np.array([robot_wrt_table[0], robot_wrt_table[1]])
            # print("lidar", pt_wrt_lidar)
            obstacles.append(pt_wrt_table)

        return obstacles

    @staticmethod
    def mask_filter_obs(obstacles: List[bool]):
        # keep obstacles inside the "table" area (taking into account edge offset)
        # [True, False] = [Obstacle in table, Obstacle outside table]
        filtered_obstacles = []
        for obs in obstacles:
            if not (obs[0] < table_x_min or obs[0] > table_x_max 
                    or obs[1] < table_y_min or obs[1] > table_y_max):
                filtered_obstacles.append(True)
            else:
                filtered_obstacles.append(False)
        return filtered_obstacles
    
    @staticmethod
    def polar_lidar_to_cartesian(polar_coord: Tuple[float, float]):
    #input : (r, theta) 
    # r = distance ; theta = angle in DEGREES
        r = polar_coord[0]
        theta = np.radians(polar_coord[1]) + np.pi/2 #adding offset
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return np.array([x, y])
    
    @staticmethod
    def translate(x, y, tx, ty):
        return (x + tx, y + ty)

    @staticmethod
    def rotate(x, y, angle): #angle in radians !
        c = np.cos(angle)
        s = np.sin(angle)
        return x * c - y * s, x * s + y * c
    
    @property
    def t_lidar_to_robot(self): #transform matrix from lidar to robot
        return np.array([
        [np.cos(self.theta_offset), -np.sin(self.theta_offset), self.x_offset],
        [np.sin(self.theta_offset), np.cos(self.theta_offset), self.y_offset],
        [0, 0, 1]
    ])

    # @staticmethod
    # @lru_cache(10)
    def t_robot_to_table(self, robot_pose: Tuple[float, float, float]):
        # transform matrix from robot to table
        rad_theta = robot_pose[2] + self.theta_offset #TODO : remove shitty offset here
        table_to_robot = np.array([
        [np.cos(rad_theta), -np.sin(rad_theta), robot_pose[0]],
        [np.sin(rad_theta), np.cos(rad_theta), robot_pose[1]],
        [0, 0, 1]])
        return np.linalg.inv(table_to_robot) #inverse table -> robot_to_table
    
    

    def _lidar_wrt_robot(self, robot_wrt_table):
        # create a homogeneous coordinate vector for this lidar point in lidar frame
        v_C_B = np.array([robot_wrt_table[0], robot_wrt_table[1], 1])
        # transform the homogeneous coordinate vector from Lidar Frame to Table Frame
        v_C_A = np.dot(self.t_lidar_to_robot, v_C_B)
        return (v_C_A[0], v_C_A[1]) 

if __name__ == "__main__":
    for t_int in range(0, 10):
        theta = 2.9 + t_int/100
        print(theta)
        a = ObstacleCalc(0.0, 0.0, theta)
    #a.calc_obstacles_wrt_table((1.84, 0.22, 0.0), 
    #                           (
    #    (1.79 ,180 + 56.52),
    #    (1.76, 282), #expected (0.05, -0.094)
    #    (2.03, 250) #expected (0.022, 1.0)
    #    )) 

        a.calc_obstacles_wrt_table((2.04, 1.098, -0.15), 
            (
            (2.07, 291.5),
            (1.50, 57.0)
            )
        ) 


    



