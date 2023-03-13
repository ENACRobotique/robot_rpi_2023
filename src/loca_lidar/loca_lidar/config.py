# general settings
Debug = False #mainly visualisation tools

# CloudPoints settings

    # obstacle cone settings
cone_angle = 120 # degrees
cone_warning_dist = 0.5 # meters
cone_stop_dist = 0.25 #meFbters

min_lidar_dist = 0.15 # Meters (radius of the robot perimeter)
max_lidar_dist = 3.0 

    #amalgames settings
# maximum squared absolute distance between two points within the cloud points to be considerered as one same amalgame
amalgame_squared_dist_max = 0.05 # meters

amalg_min_size = 0.08 # meters
amalg_max_size = 0.15 # meters
amalg_max_nb_pts = 50

# Fixed Points / Beacons coordinates
known_points_in_mm = ( #(x,y) | Made from Eurobot2023_Rules_FR_FInale, Blue Side
    (-94, 50), #A (bottom left)     | (22+22+45+5) Bordure mur + Bordure Mur + Moitié + Moitié trou
    (2094, 1500), #B (middle right)
    (-94, 2950), #C (top left)
    (1000, 3100), #D (Center of Support de Balise | Top middle)
    (225, 3100) #E (Center of Experience | top middle left)
    ) 