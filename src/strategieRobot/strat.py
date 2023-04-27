import sys, os
import time
from math import sqrt,pi 

sys.path.append(os.path.join(os.path.dirname(__file__), '../../_build'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../common'))

import generated.robot_state_pb2 as robot_pb
from strategy_test import G2S
from robot import Robot
import navigation as nav



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
        self.navigation = nav.Navigation()
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
    
    def goblue_leave(self):
        self.robot.pointsEstimes += 20
        self.robot.updateScore()

    def match_end_guard(self):
        if time.time() - self.match_start_time > MATCH_DURATION:
            return True
    
    def end_enter(self):
        self.robot.setClaw(robot_pb.SetState.ClawState.CLAW_CLOSED)
        print("This is the End!")
        exit(0)

    def dummy_tr(self):
        ...
        #print("dummy")
    
    def reset_pos(self):
        eps_degree = 5
        coeff_conv_degrad = pi / 180
        x_lidar = self.navigation.pos_x
        y_lidar = self.navigation.pos_y
        theta_lidar = self.navigation.theta
        x_odo = self.robot.x
        y_odo = self.robot.y
        theta_odo = self.robot.theta
        
        if abs(theta_lidar - theta_odo) >= eps_degree * coeff_conv_degrad:
            self.robot.theta = theta_lidar

        if sqrt((x_lidar - x_odo)**2 + (y_lidar - y_odo)**2) >= 0.05:
            self.robot.x = x_lidar
            self.robot.y = y_lidar


if __name__ == "__main__":
    parent = Parent()
    g2s = G2S(parent)
    #g2s.debug = True
    g2s.start()

    while True:
        nav.sub_obstacles.set_callback(parent.navigation.on_obstacles_received)
        nav.sub_pos.set_callback(parent.navigation.update_pos)
        g2s.check_transitions()
        time.sleep(0.2)
    

