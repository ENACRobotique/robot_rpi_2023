import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
from time import time, sleep
from math import sqrt
import sys
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

class Robot:
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
        self.pointsEstimes =0
        self.positionGriffes = "S"
        self.cerisesEnStock = 10

        self.XY_ACCURACY = 0.02  #m
        self.THETA_ACCURACY = 0.05 # plus à changer

        self.tempsDebutMatch = None


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

        self.pubSide = ProtoPublisher("side",robot_pb.Side)

        self.set_target_pos_pub = ProtoPublisher("set_position", robot_pb.Position)
        self.reset_pos_pub = ProtoPublisher("reset", robot_pb.Position)
        self.claw_pub = ProtoPublisher("set_pince", robot_pb.SetState)
        self.score_pub = ProtoPublisher("set_score", robot_pb.Match)

        
        

    def __repr__(self) -> str:
        return "Cooking Mama's status storage structure"
    
    def nextCommandNumber(self):
        if self.lastCommandNumber == None :
            return 1
        else :
            return self.lastCommandNumber +1
        
    

    def hasReachedTarget(self):
        d=sqrt((self.x-self.lastTargetX)**2 + (self.y-self.lastTargetY)**2)
        return (d <= self.XY_ACCURACY) and (abs(self.theta - self.lastTargetTheta) <= self.THETA_ACCURACY)
    

    def setTargetPos(self, x, y, theta):
        pos = robot_pb.Position(x=x, y=y, theta=theta)
        self.set_target_pos_pub.send(pos)
        self.lastTargetX = x
        self.lastTargetY = y
        self.lastTargetTheta = theta

    def resetPos(self, x, y, theta):
        self.reset_pos_pub.send(robot_pb.Position(x=x, y=y, theta=theta))

    def setClaw(self, claw_state):
        self.claw_pub.send(robot_pb.SetState(claw_state=claw_state))
    
    def updateScore(self):
        self.score_pub.send(robot_pb.Match(score=self.pointsEstimes))

    def onSetTargetPostition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise le dernier ordre de position"""
        self.lastTargetX = msg.x
        self.lastTargetY = msg.y
        self.lastTargetTheta = msg.theta

    def onReceivePosition (self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la position du robot"""
        self.x=msg.x
        self.y=msg.y
        self.theta=msg.theta
        # TODO : publish side deduced from position & if match hasn't started
        # self.pubSide.send(robot_pb.Side(side=robot_pb.Side.BLUE))
        # self.pubSide.send(robot_pb.Side(side=robot_pb.Side.GREEN))

    def onReceiveSpeed(self, topic_name, msg, timestamp):
        """Callback d'un subscriber ecal. Actualise la vitesse du robot"""
        self.Vx = msg.vx
        self.Vy = msg.vy
        self.Vtheta = msg.vtheta
    
    def onReceiveMatchStarted (self, topic_name, msg, timestamp):
        self.tempsDebutMatch = time()
        ecal_core.log_message("Match started at " + str(self.tempsDebutMatch))
    
    def onReceiveActionFinshed (self,topic_name, msg, timestamp):
        numAction = msg.action
        self.lastFinishedActionNumber = numAction


    def onProximityStatus (self,topic_name, msg, timestamp):
        self.proximityStatus = msg.status
    
    
    



    
