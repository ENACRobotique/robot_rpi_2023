import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
from time import time, sleep
from math import sqrt

class robot:
    def __init__(self,x=0.0,y=0.0,theta=0.0):
        """x et y sont en mètres
        theta est en radian"""
        self.x = x
        self.y = y
        self.theta = theta
        self.Vx =0.0
        self.Vy =0.0
        self.Vtheta=0.0

        self.lastTargetX = x
        self.lastTargetY = y
        self.lastTargetTheta = theta

        self.XY_ACCURACY = 0.01  #m
        self.THETA_ACCURACY = 0.05 # a changer

    def __repr__(self) -> str:
        return "Cooking Mama's status storage structure"

    def hasReachedTarget(self):
        d=sqrt((self.x-self.lastTargetX)**2 + (self.y-self.lastTargetY)**2)
        return (d <= self.XY_ACCURACY) and (abs(self.theta - self.lastTargetX) <= self.THETA_ACCURACY)
    

    
    
    



    
