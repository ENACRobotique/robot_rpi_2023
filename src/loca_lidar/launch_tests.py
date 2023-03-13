import pytest
import numpy as np
import loca_lidar.PatternFinder as pf
import loca_lidar.CloudPoints as cp
from  loca_lidar.PointsDataStruct import PolarPts, AmalgamePolar
import loca_lidar.config as config


######### Test amalgame discovery from cloud points ############

def test_one_amalgame():
    # Amalgame between 10° & 20°
    # Check if amalgame is detected if only the amalgame is detected by lidar and nothing else (except dummy points)
    # + test if the algorithm isn't looping
    pts_test = np.array([
        (0.01, 0),
        (0.01, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (0.01, 7),
        (0.01, 8),
        (0.01, 9),
        (1.0, 10), #expected size : 0.18
        (1.0, 11), #expected center : (1, 15)
        (1.0, 12),
        (1.0, 13),
        (1.0, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
        (0.01, 21),
        (0.01, 22),
        (0.01, 23),
        (0.01, 24),
        (0.01, 25),
        (0.01, 26),
        (0.01, 27),
    ], dtype=PolarPts)

    pts_filtered = np.array([
        (1.0, 10), #expected size : 0.18
        (1.0, 11), #expected center : (1, 15)
        (1.0, 12),
        (1.0, 13),
        (1.0, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
    ], dtype=PolarPts)
    basic_filtered = cp.basic_filter_pts(pts_test)
    assert np.array_equal(basic_filtered, pts_filtered)
    amalgames = cp.amalgames_from_cloud(basic_filtered)
    assert np.isclose(amalgames[0]['size'], 0.17431148)
    assert np.isclose(amalgames[0]['center_polar']['angle'], 15.0)
    assert np.isclose(amalgames[0]['center_polar']['distance'], 1.0)
    filtered_amalgames = cp.filter_amalgame_size(amalgames)
    assert filtered_amalgames.size == 0 #size is 0.18, above 0.15 so we shoudl'nt find amalgame

def test_continuous_amalgame(): 
    #test one amalgame present at beggining and end of scan
    #test two amalgames that are different but are next to each other in the scan
    # test one amalgame present randomly
    pts_test = np.array([
        (1.0, 0),
        (1.0, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (2.5, 7),
        (2.5, 8),
        (2.5, 9),
        (2.5, 10), #two close (when sorting by polar angle) amalgames
        (1.5, 11), 
        (1.5, 12),
        (1.5, 13),
        (1.5, 14),
        (3.50, 15),
        (3.50, 16),
        (3.50, 17),
        (3.50, 18),
        (0.7, 19), #random amalgame
        (0.7, 20),
        (0.7, 21),
        (0.7, 22),
        (0.01, 23),
        (0.01, 24),
        (0.01, 25),
        (1.0, 358),
        (1.0, 359),
    ], dtype=PolarPts)

    basic_filtered = cp.basic_filter_pts(pts_test)
    amalgames = cp.amalgames_from_cloud(basic_filtered)
    filtered_amalgames = cp.filter_amalgame_size(amalgames)
    # Test filtration :
    expected_filter = np.array([(2.5, 8.5)], dtype=PolarPts)
    assert filtered_amalgames['center_polar'] == expected_filter
    # Testing calculation of relative center : 
    tuple_amalgames = cp.amalgame_numpy_to_tuple(amalgames)
    
    # TODO : expected amalgames fill
    assert tuple_amalgames == (())

def test_continuous_amalgame_two():
    pts_test = np.array([
        (1.0, 0),
        (1.0, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (0.01, 7),
        (0.01, 8),
        (0.01, 9),
        (3.50, 10), 
        (3.50, 11), 
        (3.50, 12),
        (3.50, 13),
        (3.50, 14),
        (3.50, 15),
        (3.50, 16),
        (3.50, 17),
        (3.50, 18),
        (3.50, 19),
        (3.50, 20),
        (0.01, 21),
        (0.01, 22),
        (0.01, 23),
        (0.01, 24),
        (0.01, 25),
        (1.0, 26),
        (1.0, 27),
    ], dtype=PolarPts)

    pts_filtered = np.array([
        (1.0, 10), #expected size : 0.18
        (1.0, 11), #expected center : (1, 15)
        (1.0, 12),
        (1.0, 13),
        (1.0, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
    ], dtype=PolarPts)
    basic_filtered = cp.basic_filter_pts(pts_test)
    assert np.array_equal(basic_filtered, pts_filtered)
    amalgames = cp.amalgames_from_cloud(basic_filtered)
    empty_amalgame = np.zeros((20,), dtype=AmalgamePolar)
    assert np.array_equal(amalgames, empty_amalgame) #size is 0.18, above 0.15 so we shoudl'nt find amalgame

def test_cone_detection(): 
    pts_test = np.array([
        (0.01, 0),
        (0.01, 1),
        (0.01, 2),
        (0.01, 3),
        (0.01, 4),
        (0.01, 5),
        (0.01, 6),
        (0.01, 7),
        (0.01, 8),
        (0.01, 9),
        (1.0, 10),
        (1.0, 11),
        (1.0, 12),
        (1.0, 13),
        (0.2, 14),
        (1.0, 15),
        (1.0, 16),
        (1.0, 17),
        (1.0, 18),
        (1.0, 19),
        (1.0, 20),
        (0.01, 21),
        (0.01, 22),
        (0.01, 180),
        (0.01, 181),
        (0.01, 182),
        (0.01, 183),
        (0.01, 184),
    ], dtype=PolarPts)

    # TODO : refactor to avoid this test is dependent on config.py
    basic_filtered = cp.basic_filter_pts(pts_test)
    assert 2 == cp.obstacle_in_cone(basic_filtered, 0.0)
    assert 0 == cp.obstacle_in_cone(basic_filtered, 180.0)

######### Test finding pattern from amalgames
amalgame_sample_1 = (
    (1.57, 125.67),  
    (1.59, 237.94), 
    (1.68, 310.59), #CAREFUL : USE DEGREES ANGLE POSTIVE ONLY
    (1.62, 337.7),
    (1.57, 350.22)
)

amalgame_1 = pf.GroupAmalgame(amalgame_sample_1)

blue_beacons = pf.GroupAmalgame(tuple((x / 1000, y / 1000) for x,y in config.known_points_in_mm))

# maximum error tolerance set for unit tests is 2mm for 2D lidar pose estimation 
pos_abs_tol = 0.002 #meters 
angle_abs_tol = 0.02 #degrees
finder = pf.LinkFinder(blue_beacons.get_distances(), 0.02)


def test_all_fixed_no_obstacle_1():
    # Robot origin is at x~0.5, y~1.5, theta ~32.06° left
    # no other obstacles, approximated lidar reading from geogebra
    lidar2table = finder.find_pattern(amalgame_1.get_distances())

    # verify position
    lidar_pos = pf.Triangulate.lidar_pos_wrt_table(
        lidar2table, amalgame_sample_1, blue_beacons.points)
    expected_lidar_pos = np.array([0.5, 1.5]).reshape(2, 1)
    assert np.allclose(lidar_pos, expected_lidar_pos, atol=pos_abs_tol)

    # verify angle
    lidar_angle = pf.Triangulate.lidar_angle_wrt_table(
        lidar_pos, lidar2table, amalgame_sample_1, blue_beacons.points)
    expected_lidar_angle = 32.06
    assert np.isclose(lidar_angle, expected_lidar_angle, angle_abs_tol)

def test_partial_fixed_no_obstacle_1(): 
    #expected values :
    expected_lidar_pos = np.array([0.5, 1.5]).reshape(2, 1)
    expected_lidar_angle = 32.06

    #test amalgame_sample_1 with 4 points :
    first_four_amalgames = pf.GroupAmalgame(amalgame_sample_1[:4])
    lidar2table = finder.find_pattern(first_four_amalgames.get_distances())

    # verify position
    lidar_pos = pf.Triangulate.lidar_pos_wrt_table(
        lidar2table, first_four_amalgames.points, blue_beacons.points)
    assert np.allclose(lidar_pos, expected_lidar_pos, atol=pos_abs_tol)

    # verify angle
    lidar_angle = pf.Triangulate.lidar_angle_wrt_table(
        lidar_pos, lidar2table, first_four_amalgames.points, blue_beacons.points)
    assert np.isclose(lidar_angle, expected_lidar_angle, angle_abs_tol)

    #test amalgame_sample_1 with 3 points:
    first_three_amalgames = pf.GroupAmalgame(amalgame_sample_1[:3])
    lidar2table = finder.find_pattern(first_three_amalgames.get_distances())

    # verify position
    lidar_pos = pf.Triangulate.lidar_pos_wrt_table(
        lidar2table, first_three_amalgames.points, blue_beacons.points)
    assert np.allclose(lidar_pos, expected_lidar_pos, atol=pos_abs_tol)

    # verify angle
    lidar_angle = pf.Triangulate.lidar_angle_wrt_table(
        lidar_pos, lidar2table, first_three_amalgames.points, blue_beacons.points)
    assert np.isclose(lidar_angle, expected_lidar_angle, angle_abs_tol)

    #test amalgame_sample_1 with 2 points:
    first_two_amalgames = pf.GroupAmalgame(amalgame_sample_1[:2])
    with pytest.raises(ValueError):
        lidar2table = finder.find_pattern(first_two_amalgames.get_distances())

#TODO : check function lidar_angle_wrt_table with origin of robot aligned x or y axis to the beacon

#Test case : 2 perfect with geogebra (bottom right, top right pointing bottom left)
# take one of these and add false obstacles inside & outside 
# robot is outside 
#obstacles that make a valid positionning
#cloud points that are not valid


#def test_cloud_points():
    #assert all get_distances()
if __name__ == "__main__":
    test_one_amalgame()
    test_continuous_amalgame()
    test_continuous_amalgame_two()
    test_cone_detection()
    test_all_fixed_no_obstacle_1()
    test_partial_fixed_no_obstacle_1()
    # pytest.main(["-x", ".\\loca_lidar\\launch_tests.py"])

