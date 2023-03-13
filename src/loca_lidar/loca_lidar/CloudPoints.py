# Set of tools to manage points from 2D lidar cloud points and creation of amalgames

from math import cos, radians
import numpy as np
from loca_lidar.PointsDataStruct import PolarPts, DistPts, AmalgamePolar, AmalgameCartesian, \
    PolarPts_t, AmalgamePolar_t
import loca_lidar.config as config

def get_squared_dist_cartesian(pt1=(0,0), pt2= (0,0)): #x,y
    return (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2

#https://math.stackexchange.com/questions/1506706/how-to-calculate-the-distance-between-two-points-with-polar-coordinates
def get_squared_dist_polar(pt1, pt2):
    r1 = pt1[0]
    r2 = pt2[0]
    theta1 = pt1[1]
    theta2 = pt2[1]
    return r1**2 + r2**2 - 2 * r1 * r2 * cos(radians(theta2 - theta1))

# function to remove incoherent points from lidar scan (to pre-filter them)
def basic_filter_pts(pts: PolarPts_t) -> PolarPts_t:
    # pt should be at least within 5 cm & 3 meters distance from lidar to be in the table
    mask = (pts['distance'] < config.max_lidar_dist) & (pts['distance'] > config.min_lidar_dist)
    return pts[mask]

def position_filter_pts(pts: PolarPts_t, lidar_x, lidar_y, lidar_theta) -> PolarPts_t:
    # TODO 1. Add additionnal check with the last known lidar position on the table
    return pts

def obstacle_in_cone(pts: PolarPts_t, angle: float) -> int:
    """_summary_

    Args:
        pts (PolarPts_t): _description_
        angle (float): _description_

    Raises:
        ValueError: angle normalization problem (not in [0, 360])

    Returns:
        int: 0 : ok, 1 : warning, 2 : stop
    """
    angle_to_check = []
    half_cone = config.cone_angle / 2
    # avoiding checking negative index in pts 
    if angle >= 0 and angle <= half_cone:
        angle_to_check = [[0, angle + half_cone], [360 - half_cone + angle, 360]]
    elif angle > half_cone and angle < 360 - half_cone:
        angle_to_check = [[angle- half_cone, angle], [angle, angle + half_cone]]
    elif angle >= 360 - half_cone and angle <= 360:
        angle_to_check = [[angle - half_cone, 360], [0, angle + half_cone - 360]]
    else: 
        raise ValueError("angle value given to obstacle_in_cone is not between 0 & 360")

    #checking corresponding angle within the range given above
    obstacles = pts[np.where(
        np.logical_and(
            np.logical_or( # check valid angle
                np.logical_and(pts['angle'] > angle_to_check[0][0], pts['angle'] < angle_to_check[0][1]),
                np.logical_and(pts['angle'] > angle_to_check[1][0], pts['angle'] < angle_to_check[1][1])
            ),
            pts['distance'] < config.cone_warning_dist #check distance
        )
    )]

    # check the minimal distance from the detected obstacle in cone
    if obstacles.size == 0:
        return 0 # OK
    elif np.argmin(obstacles['distance']) < config.cone_stop_dist:
        return 2 # STOP
    else:
        return 1 # WARNING


def _fill_amalgame(amalgame:AmalgamePolar_t) -> AmalgamePolar_t:
    last_i = np.max(np.nonzero(amalgame['list_pts']['distance']))
    # Calculate amalgame relative center 
    avg_pts_dist = amalgame['list_pts']['distance'][:last_i+1].mean()
    avg_pts_angle = amalgame['list_pts']['angle'][:last_i+1].mean()
    amalgame['center_polar']['distance'] = avg_pts_dist
    amalgame['center_polar']['angle'] = avg_pts_angle

    # Calculate amalgame size :
    first_pt, last_pt = amalgame['list_pts'][0], amalgame['list_pts'][last_i]
    amalgame['size'] = np.sqrt(get_squared_dist_polar(first_pt, last_pt))
    return amalgame

def _fusion_amalgames(amalgame1:AmalgamePolar_t, amalgame2:AmalgamePolar_t, angle_norm = True) -> AmalgamePolar_t:
    # Function to manage the case when an amalgame is present at both beggining and end of scan
    # angle_norm : set to true to manage case of when angles are close to 360 and close to 0 for mean computation
    i_first_zero = np.where(amalgame1['list_pts']['distance'] == 0)[0][0]
    amalgame1['list_pts'] = np.concatenate((amalgame1['list_pts'][:i_first_zero], amalgame2['list_pts'][:-i_first_zero]))

    # recalculate size & center
    amalgame1 = _fill_amalgame(amalgame1)
    #manage case if fusion from points close to 360 AND 0 at the same time : 
    # https://stackoverflow.com/a/491784
    if angle_norm:
        last_i = np.where(amalgame1['list_pts']['distance'] == 0)[0][0]
        sum_sin = np.sum(np.sin(np.deg2rad(amalgame1['list_pts'][:last_i]['angle'])))
        sum_cos = np.sum(np.cos(np.deg2rad(amalgame1['list_pts'][:last_i]['angle'])))
        angle = np.rad2deg(np.arctan2(sum_sin, sum_cos))
        angle = angle if angle >= 0 else 360 + angle
        amalgame1['center_polar']['angle'] = angle

    return amalgame1

def amalgames_from_cloud(pts: PolarPts_t) -> AmalgamePolar_t:
    amalgames = np.zeros((30, ), dtype=AmalgamePolar) # Hypothesis that we won't detect more than 20 valid amalgames per scan
    amalg_i, amalg_pt_count = 0, 0
    #TODO : optimize below using numpy
    # Known limitation : if an amalgame made of two points is present only at the beggining and end of filtered scan, it won't be detected
    # But we consider that an amalgame should be made of at least three points
    for i, pt in enumerate(pts):
        #initialize first cur_pt
        if amalg_pt_count == 0: 
            amalgames[amalg_i]['list_pts'][amalg_pt_count] = pt
            amalg_pt_count += 1
            continue

        cur_pt = amalgames[amalg_i]['list_pts'][amalg_pt_count -1] #last added point in the amalgame

        # makes sure to finish last amalgame and adding last pt
        if i == pts.size - 1 and get_squared_dist_polar(cur_pt, pt) <= config.amalgame_squared_dist_max:
            amalgames[amalg_i]['list_pts'][amalg_pt_count] = pt

        # if next point is not part of the amalgame currently being discovered, finish it
        # or if finishing pts
        if (get_squared_dist_polar(cur_pt, pt) > config.amalgame_squared_dist_max
            or i == pts.size - 1 # this condition makes sure to finish the last amalgame if reaching the end of the pts list
            or amalg_pt_count >= config.amalg_max_nb_pts - 1): # Prevents IndexError if an amalgame is too big. 
            if amalg_pt_count >= 1:
                #calculate rel center & size : 
                amalgames[amalg_i] = _fill_amalgame(amalgames[amalg_i])
                amalg_i += 1 
            else: #we start another amalgame from the same index 
                amalgames[amalg_i] = np.zeros((1, ))
            amalg_pt_count = 1
            amalgames[amalg_i]['list_pts'][0] = pt
            continue

        # check if point could be part of the same amalgame as pt, and add it if so
        if get_squared_dist_polar(cur_pt, pt) <= config.amalgame_squared_dist_max:
            amalgames[amalg_i]['list_pts'][amalg_pt_count] = pt
            amalg_pt_count += 1
        
    # if first and last amalgame actually belong to the same amalgame, fusion them
    if amalg_i > 1: #if more than 2 amalgames detected
        #get index of last point added to amalgame
        last_i = np.max(np.nonzero(amalgames[amalg_i-1]['list_pts']['distance']))
        first_amalg_last_i = np.max(np.nonzero(amalgames[0]['list_pts']['distance']))
        first_pt_first_amalg = amalgames[0]['list_pts'][0]
        last_pt_last_amalg = amalgames[amalg_i-1]['list_pts'][last_i]
        if (get_squared_dist_polar(first_pt_first_amalg, last_pt_last_amalg) <= config.amalgame_squared_dist_max
            and first_amalg_last_i + last_i <= config.amalg_max_nb_pts - 1): # Avoid fusion of amalgame if the number of points will be too big
            amalgames[0] = _fusion_amalgames(amalgames[0], amalgames[amalg_i-1])
            amalgames[amalg_i-1] = np.zeros((1, ))

    return amalgames

def filter_amalgame_size(amalgames:AmalgamePolar_t) -> AmalgamePolar_t:
    #if amalgame is valid ("coherent size" ex :  not a wall, not a referee, ...)
    mask = np.where((amalgames['size'] > config.amalg_min_size) & (amalgames['size'] < config.amalg_max_size))
    valid_amalgames = amalgames[mask]
    return valid_amalgames

# This function filter out list of points, to only keep center of amalgames in polar coords
def amalgame_numpy_to_tuple(amalgames:AmalgamePolar_t) -> tuple():
    last_i = np.max(np.nonzero(amalgames['center_polar']['distance']))
    return tuple(amalgames[:last_i]['center_polar'])

if __name__ == '__main__':
    pass
    # get_distances(amalgame_sample_1)
    
    #print(get_distances_from_pivot(1, get_distances()))
