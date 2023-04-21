import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
from time import time, sleep
from math import sqrt
import sys
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
class robot:
    """Classe dont le but est de se subscribe à ecal pour avoir une représentation de l'état du robot
    
    Créez un objet de cette classe si vous avez besoin de connaître l'état du robot."""
    def __init__(self):
        """x et y sont en mètres
        theta est en radian"""
        ecal_core.initialize(sys.argv, "robotStateHolder")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Vx =0.0
        self.Vy =0.0
        self.Vtheta=0.0

        self.lastTargetX = 0.0
        self.lastTargetY = 0.0
        self.lastTargetTheta = 0.0

        self.proximityStatus = None

        self.lastCommandNumber = None
        self.lastFinishedActionNumber = None
        self.stockagePlateau = [0,0,0]
        self.pointsEstimés =0
        self.positionGriffes = "S"
        self.cerisesEnStock = 10

        self.XY_ACCURACY = 0.01  #m
        self.THETA_ACCURACY = 0.05 # plus à changer

        self.tempsDébutMatch = None


        self.matchReportSub = ProtoSubscriber('match_start',robot_pb.Match)
        self.matchReportSub.set_callback(self.onReceiveMatchStarted)

        self.actionReportSub = ProtoSubscriber('action',robot_pb.Action)
        self.actionReportSub.set_callback(self.onReceiveActionFinshed)

        self.positionReportSub = ProtoSubscriber("odom_pos",robot_pb.Position)
        self.positionReportSub.set_callback(self.onReceivePosition)

        self.speedReportSub = ProtoSubscriber("odom_speed",robot_pb.Speed)
        self.speedReportSub.set_callback(self.onReceiveSpeed)

        self.setPositionSub = ProtoSubscriber("set_position", robot_pb.Position)
        self.setPositionSub.set_callback(self.onSetTargetPostition)

        self.proximitySub = ProtoSubscriber("proximity_status",lidar_pb.Proximity)
        self.proximitySub.set_callback(self.onProximityStatus)
        
        

    def __repr__(self) -> str:
        return "Cooking Mama's status storage structure"
    
    def nextCommandNumber(self):
        if self.lastCommandNumber == None :
            return 1
        else :
            return self.lastCommandNumber +1
        
    

    def hasReachedTarget(self):
        d=sqrt((self.x-self.lastTargetX)**2 + (self.y-self.lastTargetY)**2)
        return (d <= self.XY_ACCURACY) and (abs(self.theta - self.lastTargetX) <= self.THETA_ACCURACY)
    
    def onSetTargetPostition (self,x,y,theta):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.lastTargetX =x
        self.lastTargetY =y
        self.lastTargetTheta =theta

    def onReceivePosition (self,x,y,theta):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        self.x=x
        self.y=y
        self.theta=theta

    def onReceiveSpeed(self,Vx,Vy,Vtheta):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.Vx = Vx
        self.Vy = Vy
        self.Vtheta = Vtheta
    
    def onReceiveMatchStarted (self):
        self.tempsDébutMatch = time()
    
    def onReceiveActionFinshed (self,numAction):
        self.lastFinishedActionNumber = numAction


    def onProximityStatus (self,status):
        self.proximityStatus =status
    
    
    



    
