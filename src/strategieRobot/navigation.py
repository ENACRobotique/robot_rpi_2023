import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import sys, os
import time 
from math import sqrt

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder

import generated.lidar_data_pb2 as lidar_pb
from robot import Robot
import map 


ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")

graph = None

sub_obstacles = ProtoSubscriber("obstacles_wrt_table", lidar_pb.Obstacles)
sub_pos = ProtoSubscriber("obstacles_wrt_table", lidar_pb.pub_lidar_pos)

def on_obstacles_received(topic_name, msg, timestamp):
    update_graph(msg.x, msg.y)

def update_graph(all_x,all_y):
    global graph
    """
    si le cercle de centre  x et y des coordonnes du robot adverse croise une des droite du graph, on rajoute +100 au weight de la droite
    """
    file = 'graph.txt'
    graph = map.read_graph(file) #map de la table
    graph.weight()

    d = 0.3 # diamètre robot adverse
    r = d/2 

    for point in graph.adj :
        for voisin in graph.adj[point]:
            x1 = graph.coords[point][0]
            y1 = graph.coords[point][1]
            x2 = graph.coords[voisin][0]
            y2 = graph.coords[voisin][1]

            for i in range(len(all_x)):
                if ( max(x1,x2) >= all_x[i] >= min(x1,x2))  or (max(y1,y2) >= all_y[i] >= min(y1,y2)) : # effectue un pré-trie pour limiter les calculs inutiles

                    a = (y2 - y1)/ (x2 - x1)
                    b = y1 - a*x1
                    segments = []
                    z = x1
                    while z <= x2 :
                        segments.append((x1, a*z + b ))
                        z += 0.02
                    for pt in segments:
                        if sqrt((all_x[i] - pt[0])**2 + (all_y[i] - pt[1])**2) <= r:
                            graph.weights[(point,voisin)] = 10 #on change le poids de l'arrête (suffisamment grand pour que ce soit impossible à choisir par dijkstra)
                            graph.weights[(voisin,point)] = 10


def reset_pos(topic_name, msg, timestamp):

if __name__ == "__main__":

    sub_obstacles.set_callback(on_obstacles_received)
    sub_pos.set_callback()
    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()