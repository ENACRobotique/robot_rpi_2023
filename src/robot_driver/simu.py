import struct
import serial
import threading
from time import time, gmtime, sleep
from enum import Enum
from math import cos, sin, atan2, sqrt, pi, radians
import generated.robot_state_pb2 as robot_pb
from ecal.core.publisher import ProtoPublisher

ROBOT_SPEED = 0.4
ROBOT_ANGULAR_SPEED = 1
DIST_PRECISION = 0.015
ANGLE_PRECISION = 0.04

class Radio(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.pos = (0, 0, 0)
        self.speed = (0, 0, 0)
        self.target_pos = (0, 0, 0)
        self.speed_factor = 1
        self.target_reached = True
        self.pos_report_time = time()
        self.step_time = time()

    def startListening (self):
        self.running =True
        self.smoothPositionPub = ProtoPublisher("smooth_pos", robot_pb.Position)
        self.start()

    def stopListening (self):
        self.running = False
    
    def run(self):
        while self.running:
            self.navigation()
            dt = time() - self.step_time
            self.step_time = time()
            self.step(dt)

            if time() - self.pos_report_time >= 0.1:
                self.pos_report_time = time()
                self.handle_pos_report(*self.pos)
                self.send_smooth_pos()
                self.handle_speed_report(*self.speed)
                self.handle_IHM(robot_pb.IHM.TIRETTE_OUT, robot_pb.IHM.BLUE, 1)
            
            sleep(0.01)
    
    def navigation(self):
        x, y, theta = self.pos
        tx, ty, ttheta = self.target_pos
        distance = sqrt((ty - y)**2 + (tx - x)**2)
        dtheta = self.normalize(ttheta - theta)
        azimut = atan2(ty - y, tx - x)
        vx, vy, vtheta = 0, 0, 0
        #print(distance)
        if distance >= DIST_PRECISION and not self.target_reached:
            vx = ROBOT_SPEED * self.speed_factor * cos(azimut)
            vy = ROBOT_SPEED * self.speed_factor * sin(azimut)
        if abs(dtheta) > ANGLE_PRECISION and not self.target_reached:
            if dtheta > 0:
                vtheta = ROBOT_ANGULAR_SPEED * self.speed_factor
            else:
                vtheta = -ROBOT_ANGULAR_SPEED * self.speed_factor
        
        self.speed = vx, vy, vtheta

    def step(self, dt):
        x, y, theta = self.pos
        vx, vy, vtheta = self.speed
        x += vx * dt
        y += vy * dt
        theta += vtheta * dt
        self.pos = x, y, theta

    def __repr__(self):
        return "Radio haut niveau"
    
    @staticmethod
    def normalize(angle):
        while angle > pi:
            angle -= 2*pi
        while angle < -pi:
            angle += 2*pi
        return angle
    
    def setTargetPosition (self, x, y, theta):
        """Sends a position command to the low level program.
        x, y, and theta are floats.
        x and y are in meters
        theta is in radians"""
        self.target_pos = x, y, theta
        self.target_reached = False
    
    def resetPosition (self, x, y, theta):
        """Resets the position of the low level program.
        x, y, and theta are floats.
        x and y are in meters
        theta is in radians"""
        self.pos = x, y, theta
    
    def sendStopSignal (self):
        """Sends a command to stop the robot."""
        self.speed_factor = 0

    def sendSlowDownSignal (self):
        """Sends a command to slow the robot down."""
        self.speed_factor = 0.5
        
    def sendResumeSignal (self):
        """Sends a command to make the robot ignore the last stop or slow command."""
        self.speed_factor = 1

    def sendCostumeSignal (self):
        """Sends a command that will activate the end of match sequence for the robot."""
        print("deploy costume")
    
    def sendClawSignal (self,claw_pos):
        """Send a command to the claws of the robot
        valueClaws is one char converted to an int.
        Accepted values are :
            'o' : open
            'g' : grab
            'c' : closed
            'C' : check cake presence
        """
        print("claws:", chr(claw_pos))

    def sendTobogganSignal (self, valueToboggan):
        """Send a command to turn the slide on or off.
        valueToboggan is a char converted to an int
        Accepted values are :
            'r' : rentré
            's' : sorti
        """
        print("toboggan:", chr(valueToboggan))

    def sendPointDisplay (self,pointNumber):
        """Sends a number to be displayed by the display
        pointNumber should be an integer smaller than 255
        """
        print("display:", pointNumber)


        

    def send_smooth_pos(self):
        x, y, theta = self.pos
        self.smoothPositionPub.send(robot_pb.Position(x=x, y=y, theta=theta))

    def handle_pos_report(self,x,y,theta):
        print("Handle_pos_report Unimplemented")

    def handle_speed_report(self,vx,vy,vtheta):
        print("Handle_speed_report Unimplemented")
    
    def handle_match_report(self):
        print("Handle_match_report Unimplemented")

    def handle_action_report(self,num):
        print("Handle_action_report Unimplemented")

    def handle_message(self,msg):
        print("Handle_message Unimplemented")
    
    def handle_checksum_error(self):
        print("Handle_checksum Unimplemented")

    def handle_cake_presence_report(self,is_cake):
        print("handle_cake_presence_report Unimplemented")

    def pince_cmd(self):
        print("Command_Pince Unimplemented")
    
    def handle_IHM(self, tirette, color, posdep):
        print("handle_IHM Unimplemented")
    


if __name__=="__main__":
    radio=Radio()
    radio.startListening()
    i=1
    
    while True:
        sleep(0.2)
        radio.sendPointDisplay(i)
        #print("send "+str(i%256))
        i+=1
