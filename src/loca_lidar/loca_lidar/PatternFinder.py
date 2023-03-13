from dataclasses import dataclass
from functools import cache, lru_cache
from itertools import combinations
from math import isclose, radians, pi, atan, degrees
import numpy as np
import time
from typing import Tuple
import logging

import loca_lidar.CloudPoints as cp
from loca_lidar.PointsDataStruct import CartesianPts, DistPts


# Regroupment of amalgame of 'same type' : fixed beacons, amalgames detected by lidar, ...
@dataclass
class GroupAmalgame: 
    points: Tuple[Tuple[float, float], ] #((x, y), ...) Coordinate of (relative) center of these amalgames

    #calculate and return the distances between all amalgames in format ((amalgame1, amalgame2, distance), ...)
    @cache
    def get_distances(self) -> tuple[DistPts]:
        temp_distances = []
        for point_combination in combinations(enumerate(self.points), 2): 
            #get all possible combination (unique permutation) of points : [((Index PT1, (x,y)), (Index Pt2, (x,y))), ...]
            index_pt1 = point_combination[0][0]
            pt1 = point_combination[0][1]
            index_pt2 = point_combination[1][0]
            pt2 = point_combination[1][1]
            squared_dist = cp.get_squared_dist_cartesian(pt1, pt2) #calculate squared distance
            temp_distances.append(DistPts(index_pt1, index_pt2, squared_dist))

        #list of all the distances possibles between the points A/0, B/1, C/2, D/3, E/4   False example : ((0, 1, 2.0), (0,2, 4.4232)
        return tuple(temp_distances)


# Function to convert from polar to cartesian coordinates
def _polar_lidar_to_cartesian(polar_coord: list[float, float]):
    #input : (r, theta) 
    # r = distance ; theta = angle in DEGREES
    r = polar_coord[0]
    theta = radians(polar_coord[1]) + pi/2 #adding offset
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y])

class LinkFinder:
    # Tools to find the possible correspondances between two GroupAmalgame (ex : beacons amalgames + lidar amalgames)
    def __init__(self, dist_pts_reference, error_margin): #DistPts, float
        self.dist_pts_reference = dist_pts_reference
        self.error_margin = error_margin
        self._generate_candidate_table_cache()
        nb_pts = np.unique(np.append(self.dist_pts_reference['pt1'],self.dist_pts_reference['pt2'])).size
        self.table_pivot_dist = {pivot_index: cp.get_distances_from_pivot(pivot_index, self.dist_pts_reference) 
            for pivot_index in range(nb_pts)}
        #example dictionnary {0: ((0, 1, 0.3242), (0, 2, 0.231)), ...}
        pass 

    def find_pattern(self, dist_pts_lidar):
        #return dict lidar_to_table association
        #calculate all amalgame distances squared and find if any distance match the ones in self.distances
        #perform a search to find possible matching distances (if nothing found, it's not a corners fixed known of the map)
        if dist_pts_lidar.size <= 1:
            raise ValueError("dist_pts_lidar size <= 1 | can't find pattern with 2 points or less")
        for detected_DistPts in dist_pts_lidar:
            #TODO : convert to binary search ?
            for candidate_table_point in self._get_candidates_table_point(detected_DistPts[2]): #check only the first point as "pivot"
                table_dists = self.table_pivot_dist[candidate_table_point]
                dists_from_pivot = cp.get_distances_from_pivot(detected_DistPts[0], dist_pts_lidar)
                
                #Test candidate_table_point - finding others distance of candidate/Beacon :
                lidar_to_table_pts = self._lidar2table_from_pivot(candidate_table_point, dists_from_pivot, table_dists)
                if len(lidar_to_table_pts) >= 3:  #at least 3 points have been "correlated" between lidar & table frame
                    return lidar_to_table_pts
                
        return None #no pattern found

    def _lidar2table_from_pivot(self, candidate_table_point, dists_from_pivot, table_dists) -> dict():
        # returns : {i_from_pivot:i_from_table, ...} 
        # where the squared_distances between the associated elements of the two arrays areclose(rtol=self.error_pargin)
        pt_lidar_to_table = {}
        for i, dist_pivot in enumerate(dists_from_pivot):
            for j, dist_table in enumerate(table_dists):
                if np.isclose(dist_pivot['squared_dist'], dist_table['squared_dist'], rtol=self.error_margin):
                    pt_lidar_to_table[dist_pivot['pt2']] = dist_table['pt2']
                    #break #Distances are non_unique and we need to match all, so keep it commented?
        if len(pt_lidar_to_table) >= 2:
            pt_lidar_to_table[candidate_table_point] = dists_from_pivot[0]['pt1']
            return pt_lidar_to_table
        return None

    @cache
    def _get_candidates_table_point(self, squared_dist)->tuple:
        #returns tuple of possible points in known_points from table reference for a certain squared_dist
        #Example : If distance is close to 2.5 m, it returns possible points (0, 1, 2)/(A,B,C)
        candidates = []
        for table_ref in self.dist_pts_reference:
            if isclose(table_ref[2], squared_dist, rel_tol=self.error_margin):
                if table_ref[0] not in candidates:
                    candidates.append(table_ref[0])
                if table_ref[1] not in candidates:
                    candidates.append(table_ref[1])
        return tuple(candidates)

    def _generate_candidate_table_cache(self):
        for squared_dist in self.dist_pts_reference['squared_dist']:
            self._get_candidates_table_point(squared_dist)


def get_distances_from_pivot(pt_index: np.int64, pt_distances: np.ndarray):
    #return all sqred_distances from the pt given by pt_index 
    #returns format : ((pt_index, other point, squared_dist)) of type DistPts
    
    a = np.where(pt_distances['pt1'] == pt_index)
    b = np.where(pt_distances['pt2'] == pt_index)
    distances_of_pivot = np.take(pt_distances,  np.unique(np.append(a, b)))  #np.unique(...) -> np.ndarray which contains all index of distances that contain at least pt_index
    
    for i in range(len(distances_of_pivot)): #'sort' the array, so that the order is for each element always (pt_index, other_pt, dist) and not (other_pt, pt_index, dist)
        if distances_of_pivot[i][0] != pt_index:
            distances_of_pivot[i] = (distances_of_pivot[i][1], distances_of_pivot[i][0], distances_of_pivot[i][2])
    return distances_of_pivot


#https://stackoverflow.com/questions/20546182/how-to-perform-coordinates-affine-transformation-using-python-part-2?answertab=votes#tab-top
def lidar_pos_wrt_table(lidar_to_table, lidar_amalgames, fixed_pts)-> tuple[float, float]:
    """returns average computed (x,y, angle) in meters, meters, radians using Least Square

    Args:
        self (_type_): _description_
        lidar_to_table (dict {int:int}): Association  of points {lidar_amalgames_index:Fixed_Point_index}
        lidar_amalgames (np.ndarray of ndtype PolarPoints): _description_
    """
    # Select correspondences
    lidar_idxs = list(lidar_to_table.keys())
    known_idxs = [lidar_to_table[i] for i in lidar_idxs]
    
    # Convert lidar coordinates to cartesian coordinates
    lidar_coords = np.array([_polar_lidar_to_cartesian(lidar_amalgames[i]) for i in lidar_idxs])
    table_coords = np.array([fixed_pts[i] for i in known_idxs])

    #determine using least square the transform equation from lidar to table
    pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])
    unpad = lambda x: x[:,:-1]
    X = pad(lidar_coords)
    Y = pad(table_coords)
    A, res, rank, s = np.linalg.lstsq(X, Y, rcond=0.01) #rcond value : Cut-off ratio
    #transform = lambda x: unpad(np.dot(pad(x), A))
    #print(f"Target : {table_coords} \n result : {transform(lidar_coords)}")
    #print( "Max error:", np.abs(table_coords - transform(lidar_coords)).max())

    lidar_wrt_table =  A[2][:2].reshape(2, 1) #calculated from least square optimisation
    return lidar_wrt_table

def lidar_angle_wrt_table(lidar_wrt_table, lidar_to_table, lidar_amalgames, fixed_pts): #TODO : fixedpts default  = fp.known_points()
    """Determine lidar angle compared to vertical axis pointing up in table/world frame
    
    Args:
        lidar_wrt_table (np.array): (x,y) of the lidar on the table/world frame
        lidar_to_table (dict): {index(lidar_amalgames) : index(fixed_pts)} association
        lidar_amalgames (tuple): (r, theta) (meters, degrees)  angles must be all positive (0-360)
        fixed_pts (_type_, optional): _description_. Defaults to FPts.known_points().
    """
    #TODO : avoid code repetition (and lru_cache it ?)
    # Select correspondences
    lidar_idxs = list(lidar_to_table.keys())
    known_idxs = [lidar_to_table[i] for i in lidar_idxs]
    
    # Convert lidar coordinates to cartesian coordinates
    lidar_polar = np.array([(lidar_amalgames[i]) for i in lidar_idxs])
    table_coords = np.array([fixed_pts[i] for i in known_idxs])

    # for each beacon :
    # calculate right triangle ABC with C lidar/beacon, A horizontal beacon, B vertical lidar 
    # we determine the angle using pythagorus, arctan(line a/line b)  ### formulas were determined "experimentaly" using geogebra
    computed_angle = []
    for i, coord in enumerate(table_coords):
        angle_lidar_beacon = lidar_polar[i][1]
        lidar_angle_wrt_table = None
        # if beacon on top left to the lidar position (<x & >y)
        if coord[0] < lidar_wrt_table[0] and coord[1] > lidar_wrt_table[1]: 
            #shape of ABC : ◥
            a = lidar_wrt_table[0] - coord[0]
            b = coord[1] - lidar_wrt_table[1]
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table =  lidar_triangle_angle + (360 - angle_lidar_beacon)

        #if beacon on top right to the lidar position (>x, >y)
        elif coord[0] > lidar_wrt_table[0] and coord[1] > lidar_wrt_table[1]:
            #shape of ABC : ◤
            a = coord[0] - lidar_wrt_table[0]
            b = coord[1] - lidar_wrt_table[1]
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table = 360 - angle_lidar_beacon - lidar_triangle_angle 

        #if beacon on bottom left to the lidar position (<x, <y)
        elif coord[0] < lidar_wrt_table[0] and coord[1] < lidar_wrt_table[1]:
            #shape of ABC : ◢
            a = lidar_wrt_table[0] - coord[0] 
            b = lidar_wrt_table[1] - coord[1]
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table = 180 - lidar_triangle_angle - angle_lidar_beacon

        #if beacon on bottom right to the lidar position (>x, <y)
        elif coord[0] > lidar_wrt_table[0] and coord[1] < lidar_wrt_table[1]:
            #shape of ABC : ◥
            #Here, A : vertical beacon, B : horizontal lidar 
            a = lidar_wrt_table[1] - coord[1] 
            b = coord[0] - lidar_wrt_table[0] 
            lidar_triangle_angle = degrees(atan(a/b))
            lidar_angle_wrt_table = 270 - lidar_triangle_angle - angle_lidar_beacon

        else: 
            logging.warning(f"lidar position {lidar_wrt_table} is perfectly aligned with a beacon - \
                can't determine angle using lidar amalgame {lidar_idxs[i]} and table fixed {known_idxs[i]}  ")

        if lidar_angle_wrt_table != None: 
            #correcting for negative angle
            lidar_angle_wrt_table = 360 + lidar_angle_wrt_table if lidar_angle_wrt_table < 0 else lidar_angle_wrt_table
            #correcting for > 360°
            lidar_angle_wrt_table = lidar_angle_wrt_table - 360 if lidar_angle_wrt_table > 360 else lidar_angle_wrt_table
            #adding to an array to return the averaged angle determined
            computed_angle.append(lidar_angle_wrt_table)

    #TODO : remove after enough testing below testing : 
    averaged_angle = np.array(computed_angle).mean()
    if np.any((computed_angle < averaged_angle - 0.5)|(computed_angle > averaged_angle + 0.5)):
        logging.warning(f"angle triangulation determined mean deviation of more than 0.5° \n \
            angles are {computed_angle} for beacons {table_coords} ")
    return np.array(computed_angle).mean()


if __name__ == "__main__":
    finder = LinkFinder(fp.known_distances(), 0.02)
    lidar2table = finder.find_pattern(cp.get_distances())
    #lidarpos = Triangulate.lidar_pos_wrt_table(lidar2table, cp.amalgame_sample_1)
    #print(Triangulate.lidar_angle_wrt_table(lidarpos, lidar2table, cp.amalgame_sample_1))
