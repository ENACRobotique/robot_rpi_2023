import sys, os
import time
from math import sqrt,pi, radians

sys.path.append(os.path.join(os.path.dirname(__file__), '../../_build'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))

import generated.robot_state_pb2 as robot_pb
from strategy_test import G2S
from robot import Robot, TEMPS_MINIMAL_TRIGGER, TEMPS_MAXIMAL_RECALLAGE, RecallageEtat
#import navigation as nav

MATCH_DURATION = 95

DB = {
    "INIT_POS" : (0.160, 0.225, radians(135)),

    "POS1": (0.500, 0.650, radians(135)), #croisement_NW
    "POS2" : (0.75, 0.550, radians(135)), #
    "POS3" : (1.200, 0.225, radians(135)),
    "POS_PLATE_GREEN" : (1.200, 0.225, 0),

    "POS_PUSH_CAKE" : (0.5, 0.225, 0),
    "POS_PUSH_CAKE_DONE" : (0.7, 0.225, 0),

    "POS_TOWARD_MARRON": (1.125, 0.400, -radians(90)),
    "POS_MARRON": (1.125, 0.56, -radians(90)),

    "POS_PLATE_BLUE2" : (1.850, 0.25, 0),
    "POS_TEST" : (2.00, 1.50, 0),
}

DG = {
    "INIT_POS" : (0.160, 1.775, radians(135)),

    "POS1": (0.500, 1.350, radians(135)), #croisement_NE
    "POS2" : (0.75, 1.450, radians(135)),  
    "POS3" : (1.200, 1.775, radians(135)),
    "POS_PLATE_GREEN" : (1.200, 1.775, 0),

    "POS_PUSH_CAKE" : (0.5, 1.775, 0),
    "POS_PUSH_CAKE_DONE" : (0.7, 1.775, 0),

    "POS_TOWARD_MARRON": (1.125, 1.600, radians(90)),
    "POS_MARRON": (1.125, 1.44, radians(90)),

    "POS_PLATE_BLUE2" : (1.850, 1.700, 0),
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
        self.robot.setClaw(robot_pb.SetState.CLAW_CLOSED)# type: ignore
        self.robot.setTobogganState(robot_pb.SetState.TOBOGGAN_CLOSED)
        local.initEnterTime=time.time()
        local.increment =1


    def init_loop(self,local):
        if (local.increment*1 + local.initEnterTime) < time.time():
            self.robot.pointsEstimes = (80 if self.robot.color==robot_pb.IHM.BLUE else 60) + (2 if local.increment%2 ==0 else 1)
            local.increment+=1
            self.robot.updateScore()
        
    def init_leave(self,local,next_state):
        self.robot.pointsEstimes = 5
        self.match_start_time = time.time()
        if self.robot.color == robot_pb.IHM.BLUE:       #en fonction de l'état de l'interrupteur pour choisir le côté
            self.d = DB
        else:
            self.d = DG
        self.robot.resetPos(*self.d["INIT_POS"])
        print("init leave")
        
    def match_started(self,local):
        # self.tempsDebutMatch is not None
        if self.robot.tirette == robot_pb.IHM.TIRETTE_OUT:
            print("oh boy")
            return True
    
    def cerise_enter(self,local,previous_state):
        self.robot.setTobogganState(robot_pb.SetState.TOBOGGAN_OPEN)
        local.toboggan_open_time = time.time()
    
    def cerise_leave(self,local,next_state):
        self.robot.setTobogganState(robot_pb.SetState.TOBOGGAN_CLOSED)
        self.robot.pointsEstimes+=15
        self.robot.cerisesEnStock =0
        self.robot.updateScore()

    def cerise_dropped(self, local):
        return time.time() - local.toboggan_open_time > 2
    
    def gogreen_enter(self,local,previous_state):
        print("gogreen_enter")
        self.robot.setTargetPos(*self.d["POS1"])
        local.green_substate = 0

    def loop_gogreen(self,local):
        if local.green_substate == 0:
            if self.robot.hasReachedTarget():
                print("POS1 reached, goto POS2")
                self.robot.setTargetPos(*self.d["POS2"])
                self.robot.updateScore()
                local.green_substate += 1
        elif local.green_substate == 1:
            if self.robot.hasReachedTarget():
                print("POS2 reached, goto POS3")
                self.robot.setTargetPos(*self.d["POS3"])
                local.green_substate += 1
        elif local.green_substate == 2:
            if self.robot.hasReachedTarget():
                print("POS3 reached, goto POS_PLATE_GREEN")
                self.robot.setTargetPos(*self.d["POS_PLATE_GREEN"])
                local.green_substate += 1
        elif local.green_substate == 3:
            if self.robot.hasReachedTarget():
                print("POS_PLATE_GREEN reached. Claw: ", robot_pb.SetState.CLAW_OPEN) # type: ignore
                self.robot.setClaw(robot_pb.SetState.CLAW_OPEN)# type: ignore
                local.claw_open_time = time.time()
                local.green_substate += 1
            else:
                print(f"waiting to reach POS_PLATE_GREEN. pos: {self.robot.x}, {self.robot.y}, {self.robot.theta}")
        elif local.green_substate == 4:
            if time.time() - local.claw_open_time > 0.5:
                local.green_substate += 1 

    def at_green(self,local):
        return local.green_substate == 5
    
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
        self.robot.setTargetPos(*self.d["POS_PLATE_GREEN"])
        local.substate = 0
    
    def pushcake_loop(self, local):
        if local.substate == 0:
            if self.robot.hasReachedTarget():
                self.robot.setTargetPos(*self.d["POS_PUSH_CAKE"])
                local.substate += 1
        elif local.substate == 1:
            if self.robot.hasReachedTarget():
                print("POS_PUSH_CAKE reached, goto POS_PUSH_CAKE_DONE")
                self.robot.setTargetPos(*self.d["POS_PUSH_CAKE_DONE"])
                local.substate += 1

    def cake_pushed(self,local):
        return local.substate == 2 and self.robot.hasReachedTarget()

    def pushcake_leave(self,local,next_state):
        print("pushcake leave")
        #self.robot.setClaw(robot_pb.SetState.CLAW_CLOSED)
        self.robot.pointsEstimes += 6
        self.robot.updateScore()

    def marron_enter(self, local, prev):
        print("enter marron, goto POS_TOWARD_MARRON")
        local.substate = 0
        self.robot.setTargetPos(*self.d["POS_TOWARD_MARRON"])
        self.robot.setClaw(robot_pb.SetState.CLAW_OPEN)

    def marron_loop(self, local):
        if local.substate == 0:
            if self.robot.hasReachedTarget():
                #print("POS_PLATE_GREEN reached, goto POS_TOWARD_MARRON")
                local.substate += 1
                #self.robot.setTargetPos(*self.d["POS_TOWARD_MARRON"])
                #self.robot.setClaw(robot_pb.SetState.CLAW_OPEN)
        elif local.substate == 1:
            if self.robot.hasReachedTarget():
                local.substate += 1
        elif local.substate == 2:
            if self.robot.recallage() == RecallageEtat.OK:
                print("recalage at POS_TOWARD_MARRON, goto POS_TOWARD_MARRON")###
                local.substate += 1
                self.robot.setTargetPos(*self.d["POS_TOWARD_MARRON"])
        elif local.substate == 3:
            if self.robot.hasReachedTarget():
                print("POS_TOWARD_MARRON reached, goto POS_MARRON")
                local.substate += 1
                self.robot.setTargetPos(*self.d["POS_MARRON"])
        elif local.substate == 4:
            if self.robot.hasReachedTarget():
                print("POS_MARRON reached, goto POS_TOWARD_MARRON")
                local.substate += 1
                self.robot.setClaw(robot_pb.SetState.CLAW_GRAB)
                local.time_claw = time.time()
        elif local.substate == 5:
            if time.time() - local.time_claw > 0.5:
                local.substate += 1
                self.robot.setTargetPos(*self.d["POS_TOWARD_MARRON"])
        elif local.substate == 6:
            if self.robot.hasReachedTarget():
                print("POS_TOWARD_MARRON reached, goto POS_PLATE_GREEN")
                local.substate += 1
                self.robot.setTargetPos(*self.d["POS_PLATE_GREEN"])
        elif local.substate == 7:
            if self.robot.hasReachedTarget():
                print("POS_PLATE_GREEN reached, goto POS_PUSH_CAKE")
                local.substate += 1
                self.robot.setTargetPos(*self.d["POS_PUSH_CAKE"])
                self.robot.setClaw(robot_pb.SetState.CLAW_OPEN)
        elif local.substate == 8:
            if self.robot.hasReachedTarget():
                print("POS_PUSH_CAKE reached, goto POS_PUSH_CAKE_DONE")
                local.substate += 1
                self.robot.setTargetPos(*self.d["POS_PUSH_CAKE_DONE"])
        elif local.substate == 9:
            if self.robot.hasReachedTarget():
                print("POS_PUSH_CAKE_DONE reached.")
                local.substate += 1
                self.robot.setClaw(robot_pb.SetState.CLAW_CLOSED)


    def marron_grabbed(self, local):
        return local.substate == 10
    
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
        self.robot.setClaw(robot_pb.SetState.CLAW_CLOSED)# type: ignore
        self.robot.sendCostumeSignal()
        local.t = time.time()
    
    def end_loop(self, local):
        if time.time() - local.t > 1:
            self.robot.sendCostumeSignal()
            local.t = time.time()
            print("This is the End!")


if __name__ == "__main__":
    parent = Parent()
    g2s = G2S(parent)
    g2s.debug = False
    g2s.start()

    while True:
        #nav.sub_obstacles.set_callback(parent.navigation.on_obstacles_received)
        #nav.sub_pos.set_callback(parent.navigation.update_pos)
        g2s.step()
        time.sleep(0.2)
    

