# general settings
Debug = False #mainly visualisation tools

#Check_obstacle settings
# !! Careful : offset are not implemented yet (value different from 0 may not work)
lidar_x_offset = 0.0
lidar_y_offset = 0.0
lidar_theta_offset = 0.0

# 10 cm offset from table edge
table_x_min = 0.1 # meters
table_x_max = 1.9 # meters
table_y_min = 0.1 # meters
table_y_max = 2.9 # meters
# CloudPoints settings

# obstacle cone settings
cone_angle = 120 # degrees
cone_warning_dist = 0.5 # meters
cone_stop_dist = 0.25 #meFbters

min_lidar_dist = 0.15 # Meters (radius of the robot perimeter)
max_lidar_dist = 3.0 

    #amalgames settings
# maximum squared absolute distance between two points within the cloud points to be considerered as one same amalgame
amalgame_squared_dist_max = 0.025 # meters

amalg_min_size = 0.06 # meters # min size of detected pylones of 0.1m
amalg_max_size = 0.41 # meters  # max calculable size for centrale pylone
# max size for pylones is 0.1m
amalg_max_nb_pts = 50

# Fixed Points / Beacons coordinates
known_points_in_mm = ( #(x,y) | Made from Eurobot2023_Rules_FR_FInale, Blue Side
    (-94, 50), #A (bottom left)     | (22+22+45+5) Bordure mur + Bordure Mur + Moitié + Moitié trou
    (2094, 1500), #B (middle right)
    (-94, 2950), #C (top left)
    (1000, 3100), #D (Center of Support de Balise | Top middle)
    (225, 3100) #E (Center of Experience | top middle left)
    ) 