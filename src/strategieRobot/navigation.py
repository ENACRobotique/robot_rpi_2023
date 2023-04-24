
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher, StringPublisher
import sys 
import time 
import loca_lidar.loca_lidar.robot_state_pb2 as robot_pb
import loca_lidar.launch_loca_ecal as localidar
import map 
from math import sqrt 
ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")


sub_obstacles = ProtoSubscriber("obstacle", robot_pb.Position)

def update_graph(x,y):
    """
    si le cercle de centre  x et y des coordonnes du robot adverse croise une des droite du graph, on rajoute +100 au weight de la droite
    """
    file = 'src\graph.txt'
    graph = map.read_graph(file) #map de la table

    d = 0.3 # diamètre robot adverse
    r = d/2 

    for point in graph.adj :
        for voisin in graph.adj[point]:
            x1 = graph.coords[point][0]
            y1 = graph.coords[point][1]
            x2 = graph.coords[voisin][0]
            y2 = graph.coords[voisin][1]

            a = (y2 - y1)/ (x2 - x1)
            b = y1 - a*x1
            segments = []
            z = x1
            while z <= x2 :
                segments.append((x1, a*z + b ))
                z += 0.01
            for point in segments:
                if sqrt((x - point[0])**2 + (y - point[1])**2) <= r:
                    pass #on change le poids de l'arrête
                    





if __name__ == "__main__":

    sub_obstacles.set_callback(update_graph)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()