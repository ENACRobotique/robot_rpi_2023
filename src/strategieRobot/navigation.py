import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import sys, os
import time 
from math import sqrt

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder

import generated.lidar_data_pb2 as lidar_pb
from robot import Robot
import generated.robot_state_pb2 as robot_pb
import map 

ecal_core.initialize(sys.argv, "loca_lidar_ecal_interface")
sub_obstacles = ProtoSubscriber("obstacles_wrt_table", lidar_pb.Obstacles)
sub_pos = ProtoSubscriber("obstacles_wrt_table", robot_pb.Position)


class Navigation:

    def __init__(self):
        self.file = 'graph.txt'
        self.graph = map.read_graph(self.file) #map de la table
        self.graph.weight()
        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0


    def on_obstacles_received(self, topic_name, msg, timestamp):
        self.update_graph(msg.x,msg.y)

    def update_graph(self,all_x,all_y):
        """
        si le cercle de centre  x et y des coordonnes du robot adverse croise une des droite du graph, on rajoute +100 au weight de la droite
        """
        d = 0.3 # diamètre robot adverse
        r = d/2 

        for point in self.graph.adj :
            for voisin in self.graph.adj[point]:
                x1 = self.graph.coords[point][0]
                y1 = self.graph.coords[point][1]
                x2 = self.graph.coords[voisin][0]
                y2 = self.graph.coords[voisin][1]
                w =  sqrt((x1 - x2)**2 + (y1 - y2)**2)
                self.graph.weights[(point,voisin)] = w
                self.graph.weights[(voisin,point)] = w

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
                                self.graph.weights[(point,voisin)] = 10 #on change le poids de l'arrête (suffisamment grand pour que ce soit impossible à choisir par dijkstra)
                                self.graph.weights[(voisin,point)] = 10

    def update_pos(self,topic_name, msg, timestamp):
        """
        Réévalue la position du robot après qu'il se soit déplacé, grâce au lidar
        """
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.theta = msg.theta


ecal_core.finalize()