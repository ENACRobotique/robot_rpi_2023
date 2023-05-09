import sys, os
import time
from math import sqrt,pi 

sys.path.append(os.path.join(os.path.dirname(__file__), '../../_build'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))

import generated.robot_state_pb2 as robot_pb
from strategy_test import G2S
from robot import Robot, TEMPS_MINIMAL_TRIGGER, TEMPS_MAXIMAL_RECALLAGE, RecallageEtat
#import navigation as nav

MATCH_DURATION = 100

DB = {
    "POS1": (0.500, 0.650, 0), #croisement_NW
    "POS2" : (0.95, 0.650, 0), #
    "POS_PLATE_GREEN" : (1.150, 0.225, 0),
    "POS_PUSH_CAKE" : (0.5, 0.225, 0),
    "POS_PLATE_BLUE2" : (1.850, 0.3, 0),
    "INIT_POS" : (0.225, 0.225, 0),
    "POS_TEST" : (2.00, 1.50, 0),
}

DG = {
    "POS1": (0.500, 1.3875), #croisement_NE
    "POS2" : (0.95, 1.150, 0),  
    "POS_PLATE_GREEN" : (1.150, 1.775, 0),
    "POS_PUSH_CAKE" : (0.5, 1.775, 0),
    "POS_PLATE_BLUE2" : (1.850, 1.775, 0),
    "INIT_POS" : (0.225, 0.225, 0),
    "POS_TEST" : (2.00, 1.50, 0),
}

class Parent:
    def __init__(self):
        self.robot =Robot()        
        #self.navigation = nav.Navigation()
        self.match_start_time = time.time()
        self.robot.pointsEstimes = 5  # panier présent
        time.sleep(1)
 #       self.forced_recallage_state = 0
        
    def init_enter(self,local,previous_state):
        print("init enter")
        local.wake = time.time()
        #tdself.robot.resetPos(*INIT_POS)
        self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_CLOSED)# type: ignore
        
    def init_leave(self,local,next_state):
        self.match_start_time = time.time()
        print("init leave")
        
    def match_started(self,local):
        print(self.robot.x)
        # self.tempsDebutMatch is not None
        if time.time() - local.wake > 2:    #remplacer par tirette !!!
            local.toboggan_open_time = time.time()
            self.robot.setTobogganState(robot_pb.SetState.TobogganState.TOBOGGAN_OPEN) # type: ignore
            if time.time() - local.toboggan_open_time > 2: #attend 2s avant de remonter le toboggan
                self.robot.setTobogganState(robot_pb.SetState.TobogganState.TOBOGGAN_CLOSED) # type: ignore
            print("oh boy")
            if False:       #en fonction de l'état de l'interrupteur pour choisir le côté
                self.d = DB
            else:
                self.d = DG
            return True
    
    def gogreen_enter(self,local,previous_state):
        print("gogreen_enter")
        self.robot.setTargetPos(*self.d["POS1"])
        local.green_substate = 0

    def loop_gogreen(self,local):
        match local.green_substate:
            case 0:
                if self.robot.hasReachedTarget():
                    local.green_substate = 1
                    self.robot.setTargetPos(*self.d["POS2"])
                    self.robot.updateScore()
            case 1:
                if self.robot.hasReachedTarget():
                    local.green_substate = 2
                    self.robot.setTargetPos(*self.d["POS_PLATE_GREEN"])
            case 2:
                if self.robot.hasReachedTarget():
                    print("claw: ", robot_pb.SetState.ClawState.CLAW_OPEN) # type: ignore
                    self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_OPEN)# type: ignore
                    local.claw_open_time = time.time()
                    local.green_substate = 3
            case 3:
                   if time.time() - local.claw_open_time > 0.5:
                     local.green_substate = 4  

    def at_green(self,local):
        return local.green_substate == 4 
    
    def recal_ok(self,local):
        if (time.time()- local.temps_debut_recal) >  TEMPS_MAXIMAL_RECALLAGE :
            return self.robot.recallage(forced=True, trigger=False)== RecallageEtat.OK
        elif(time.time()- local.temps_debut_recal) >  TEMPS_MINIMAL_TRIGGER:
            return self.robot.recallage(forced=False, trigger=True) == RecallageEtat.OK
        return self.robot.recallage() == RecallageEtat.OK

    def debut_recal(self,local,previous_state):
        local.temps_debut_recal = time.time()
        

    # def recal_forced_ok(self):
    #     print(self.forced_recallage_state)
    #     if self.forced_recallage_state == 0:
    #         self.time_forced_recallage = time.time()
    #         self.forced_recallage_state = 1

    #     elif self.forced_recallage_state == 1:
    #         if time.time() - self.time_forced_recallage > 3:
    #             self.forced_recallage_state = 2
    #         return self.robot.recallage() == RecallageEtat.OK

    #     elif self.forced_recallage_state == 2 :
    #         x,y,theta = self.robot.best_lidar_trigger_move()
    #         self.robot.setTargetPos(x,y,theta)
    #         self.forced_recallage_state = 3
        
    #     elif self.forced_recallage_state == 3 :
    #         if self.robot.hasReachedTarget():
    #             self.forced_recallage_state = 0

    
    def pushcake_enter(self,local,previous_state):
        self.robot.setTargetPos(*self.d["POS_PUSH_CAKE"])

    def cake_pushed(self,local):
        return self.robot.hasReachedTarget()

    def pushcake_leave(self,local,next_state):
        self.robot.pointsEstimes += 6
        self.robot.updateScore()
    
    def goblue_enter(self,local,RecalCake):
        self.robot.setTargetPos(*self.d["POS_PLATE_BLUE2"])

    def at_blue(self,local):
        return self.robot.hasReachedTarget()
    
    def goblue_leave(self,local,next_state):
        self.robot.pointsEstimes += 15
        self.robot.updateScore()

    def match_end_guard(self,local):
        if time.time() - self.match_start_time > MATCH_DURATION:
            return True
    
    def end_enter(self,local,previous_state):
        self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_CLOSED)# type: ignore
        self.robot.sendCostumeSignal()
        print("This is the End!")
        exit(0)

    def store_cake_enter(self,local,previous_state):
        self.robot.storeDisk()

    def dropDisk_enter(self,local,previous_state):
        self.robot.dropDiskFromStorage()

if __name__ == "__main__":
    parent = Parent()
    g2s = G2S(parent)
    g2s.debug = True
    g2s.start()

    while True:
        #nav.sub_obstacles.set_callback(parent.navigation.on_obstacles_received)
        #nav.sub_pos.set_callback(parent.navigation.update_pos)
        g2s.step()
        time.sleep(0.2)
    

