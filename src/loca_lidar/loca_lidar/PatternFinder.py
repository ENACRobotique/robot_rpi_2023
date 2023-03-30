from dataclasses import dataclass
from functools import cache, lru_cache, cached_property
from itertools import combinations, chain
from math import isclose, radians, pi, atan, degrees
import numpy as np
import time
from typing import Tuple, NamedTuple
import logging

import loca_lidar.CloudPoints as cp

DistPts = NamedTuple('DistPts', [
    ('index_pt1', int), 
    ('index_pt2', int), 
    ('sqrd_dist', float)])


# Regroupment of amalgame of 'same type' : fixed beacons, amalgames detected by lidar, ...
@dataclass
class GroupAmalgame: 
    points: Tuple[Tuple[float, float], ...] #((x, y), ...) Coordinate of (relative) center of these amalgames
    cartesian: bool = True #True if points are in cartesian coordinates, False if in polar coordinates
    @cached_property
    #calculate and return the distances between all amalgames in format ((amalgame1, amalgame2, distance), ...)
    def distances(self) -> tuple[DistPts]:
        temp_distances = []
        for point_combination in combinations(enumerate(self.points), 2): 
            #get all possible combination (unique permutation) of points : [((Index PT1, (x,y)), (Index Pt2, (x,y))), ...]
            index_pt1 = point_combination[0][0]
            pt1 = point_combination[0][1]
            index_pt2 = point_combination[1][0]
            pt2 = point_combination[1][1]
            if self.cartesian:
                squared_dist = cp.get_squared_dist_cartesian(pt1, pt2) #calculate squared distance
            else:
                squared_dist = cp.get_squared_dist_polar(pt1, pt2)
            temp_distances.append(DistPts(index_pt1, index_pt2, squared_dist))

        #list of all the distances possibles between the points A/0, B/1, C/2, D/3, E/4   False example : ((0, 1, 2.0), (0,2, 4.4232)
        return tuple(temp_distances)
    
# Function to convert from polar to cartesian coordinates
def polar_lidar_to_cartesian(polar_coord: list[float]): # distance, degrees
    #input : (r, theta) 
    # r = distance ; theta = angle in DEGREES
    r = polar_coord[0]
    theta = radians(polar_coord[1]) + pi/2 #adding offset
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y])

def angle_3_pts(a: tuple, b: tuple, c:tuple, is_cartesian = True)-> float:
    # return the angle between 3 points in degrees
    if not is_cartesian:
        b = polar_lidar_to_cartesian(b) # type: ignore
        a = polar_lidar_to_cartesian(a) # type: ignore
        c = polar_lidar_to_cartesian(c) # type: ignore
    angle = np.arctan2(c[1] - b[1], c[0] - b[0]) - \
                np.arctan2(a[1] - b[1], a[0] - b[0])
    return np.rad2deg(angle)

def angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return np.arctan2(dy, dx)

class LinkFinder:
    # Tools to find the possible correspondances between two GroupAmalgame (ex : beacons amalgames + lidar amalgames)
    # TODO : Filter the possible correspondances with angles known from the beacons and must be present beacons (experience or poteau fixe)
    def __init__(self, amalg, error_margin): #DistPts, float
        self.table = amalg
        self.dist_pts_reference = amalg.distances
        self.error_margin = error_margin
        self._generate_candidate_table_cache()
        nb_pts = len(set(chain.from_iterable(
            (x.index_pt1, x.index_pt2) for x in self.dist_pts_reference)
        )) # get the number of distinct points in the dist_pts
        
        self.table_pivot_dist = {pivot_index: self.get_distances_from_pivot(pivot_index, self.dist_pts_reference) 
            for pivot_index in range(nb_pts)}
        #example dictionnary {0: ((0, 1, 0.3242), (0, 2, 0.231)), ...}
        pass 

    def find_pattern(self, amalg):
        #return dict lidar_to_table association
        #calculate all amalgame distances squared and find if any distance match the ones in self.distances
        #perform a search to find possible matching distances (if nothing found, it's not a corners fixed known of the map)
        dist_pts_lidar = amalg.distances
        lidar_to_table_corr = []
        if len(dist_pts_lidar) <= 1:
            raise ValueError("dist_pts_lidar size <= 1 | can't find pattern with 2 points or less")
        for detected_DistPts in dist_pts_lidar:
            #TODO : convert to binary search ?
            for candidate_table_point in self._get_candidates_table_point(detected_DistPts[2]): #check only the first point as "pivot"
                table_dists = self.table_pivot_dist[candidate_table_point]
                dists_from_pivot = self.get_distances_from_pivot(detected_DistPts[0], dist_pts_lidar)
                
                #Test candidate_table_point - finding others distance of candidate/Beacon :
                lidar_to_table_pts = self._lidar2table_from_pivot(candidate_table_point, dists_from_pivot, table_dists)
                if lidar_to_table_pts is not None and len(lidar_to_table_pts) >= 3:  #at least 3 points have been "correlated" between lidar & table frame
                    lidar_to_table_corr.append(lidar_to_table_pts)
        print(lidar_to_table_corr)
        print(self._filter_candidate_corr(lidar_to_table_corr, amalg))
        return self._filter_candidate_corr(lidar_to_table_corr, amalg)
                
        return None #no pattern found
    def _filter_candidate_corr(self, lidar_to_table_corr, amalg): #list[dict(correspondances)]
        #TODO : Filter the possible correspondances with angles known from the beacons and must be present beacons (experience or poteau fixe)
        if len(lidar_to_table_corr) == 0:
            return None
        
        # Filter the possible correspondances with angles known from the beacons :
        pts = amalg.points
        if not amalg.cartesian:
            pts = [polar_lidar_to_cartesian(p) for p in pts]
        # 1) Reference Direction angle 
        # computes the reference direction and angle (ref_angle) using the lidar and table points with the smallest x-coordinate. 
        table_ref_index = min(range(len(self.table.points)), key=lambda i: self.table.points[i][0])
        lidar_ref_index = min(range(len(pts)), key=lambda i: pts[i][0])
        lidar_ref_point = pts[lidar_ref_index]
        table_ref_point = self.table.points[table_ref_index]
        ref_angle = angle(lidar_ref_point, table_ref_point)

        threshold = np.deg2rad(5.0) # (1.5)
        for possible_corr in lidar_to_table_corr:
            angles = {}
            for lidar_index, table_index in possible_corr.items():
                lidar_point = pts[lidar_index]
                table_point = self.table.points[table_index]
                angles[lidar_index] = angle(lidar_ref_point, lidar_point) - angle(table_ref_point, table_point) - ref_angle
            valid_corr = {lidar_index: table_index for lidar_index, table_index in possible_corr.items() if abs(angles[lidar_index]) < threshold}
            pass
        return max(lidar_to_table_corr, key=len)

    def _lidar2table_from_pivot(self, candidate_table_point, dists_from_pivot, table_dists) -> dict():
        # returns : {i_from_pivot:i_from_table, ...} 
        # where the squared_distances between the associated elements of the two arrays areclose(rtol=self.error_pargin)
        pt_lidar_to_table = {}
        for i, dist_pivot in enumerate(dists_from_pivot):
            for j, dist_table in enumerate(table_dists):
                if np.isclose(dist_pivot.sqrd_dist, dist_table.sqrd_dist, rtol=self.error_margin):
                    pt_lidar_to_table[dist_pivot.index_pt2] = dist_table.index_pt2
                    #break #Distances are non_unique and we need to match all, so keep it commented?
        if len(pt_lidar_to_table) >= 2:
            #add the pivot point :
            lidar_i = dists_from_pivot[0].index_pt1
            pt_lidar_to_table[lidar_i] = candidate_table_point

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
        for dist_pts in self.dist_pts_reference:
            self._get_candidates_table_point(dist_pts.sqrd_dist)

    @staticmethod
    def get_distances_from_pivot(pt_index: np.int64, pt_distances: list[DistPts]) -> list[DistPts]:
        #return all sqred_distances from the pt given by pt_index 
        #returns format : ((pt_index, other point, squared_dist)) of type DistPts
        
        distances_of_pivot = []
        for pt_dist in pt_distances:
            if pt_dist.index_pt1 == pt_index:
                distances_of_pivot.append(DistPts(pt_dist[0], pt_dist[1], pt_dist[2]))
            elif pt_dist.index_pt2 == pt_index:
                distances_of_pivot.append(DistPts(pt_dist[1], pt_dist[0], pt_dist[2]))  #'sort' the array, so that the order is for each element always (pt_index, other_pt, dist) and not (other_pt, pt_index, dist)

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
    lidar_coords = np.array([polar_lidar_to_cartesian(lidar_amalgames[i]) for i in lidar_idxs])
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
    if np.any((computed_angle < averaged_angle - 1.5)|(computed_angle > averaged_angle + 1.5)):
        logging.warning(f"angle triangulation determined mean deviation of more than 1.5° \n \
            angles are {computed_angle} for beacons {table_coords} ")
    return np.array(computed_angle).mean()


if __name__ == "__main__":
    pass
    # finder = LinkFinder(fp.known_distances(), 0.02)
    # lidar2table = finder.find_pattern(cp.distances)
    #lidarpos = Triangulate.lidar_pos_wrt_table(lidar2table, cp.amalgame_sample_1)
    #print(Triangulate.lidar_angle_wrt_table(lidarpos, lidar2table, cp.amalgame_sample_1))
