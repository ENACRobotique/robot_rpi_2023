import sys, os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../_build'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))

import generated.robot_state_pb2 as robot_pb
from strategy_test import G2S
from robot import Robot


MATCH_DURATION = 100

INIT_POS = 0.225, 0.225, 0
POS1 = 0.500, 0.650, 0
POS2 = 1.00, 0.650, 0
POS_PLATE_GREEN = 1.150, 0.225, 0
POS_PUSH_CAKE = 0.5, 0.225, 0
POS_PLATE_BLUE2 = 1.850, 0.225, 0

class Parent:
    def __init__(self):
        self.robot =Robot()
        self.wake = time.time()
        self.match_start_time = time.time()
        self.robot.pointsEstimes = 5  # panier présent
        time.sleep(1)
        self.go_green_substate = 0
        
    def init_enter(self):
        print("init enter")
        self.robot.resetPos(*INIT_POS)
        self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_CLOSED)
        
    def init_leave(self):
        self.match_start_time = time.time()
        print("init leave")
        
    def match_started(self):
        print(self.robot.x)
        # self.tempsDebutMatch is not None
        if time.time() - self.wake > 2:
            print("oh boy")
            return True
    
    def gogreen_enter(self):
        print("gogreen_enter")
        self.robot.setTargetPos(*POS1)
        pass

    def at_green(self):
        if self.robot.hasReachedTarget():
            if self.go_green_substate == 0:
                self.go_green_substate = 1
                self.robot.setTargetPos(*POS2)
                self.robot.updateScore()
            elif self.go_green_substate == 1:
                self.go_green_substate = 2
                self.robot.setTargetPos(*POS_PLATE_GREEN)
            elif self.go_green_substate == 2:
                print("claw: ", robot_pb.SetState.ClawState.CLAW_OPEN)
                self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_OPEN)
                self.claw_open_time = time.time()
                self.go_green_substate = 3
            else:
                if time.time() - self.claw_open_time > 0.5:
                    return True
    
    def pushcake_enter(self):
        self.robot.setTargetPos(*POS_PUSH_CAKE)

    def cake_pushed(self):
        return self.robot.hasReachedTarget()

    def pushcake_leave(self):
        self.robot.pointsEstimes += 6
        self.robot.updateScore()
    
    def goblue_enter(self):
        self.robot.setTargetPos(*POS_PLATE_BLUE2)
        
    def at_blue(self):
        return self.robot.hasReachedTarget()

    def match_end_guard(self):
        if time.time() - self.match_start_time > MATCH_DURATION:
            return True
    
    def end_enter(self):
        self.robot.pointsEstimes += 20
        self.robot.updateScore()
        self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_CLOSED)
        print("This is the End!")
        exit(0)

    def dummy_tr(self):
        ...
        #print("dummy")


parent = Parent()
g2s = G2S(parent)
#g2s.debug = True
g2s.start()

while True:
    g2s.check_transitions()
    time.sleep(0.2)
