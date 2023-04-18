import serial
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
import sys, time

import robot_state_pb2 as robot_pb
import robot_driver as rb

class Robot_Driver():
    def __init__(self, port='COM2', baudrate=115200) -> None:
        # init serial com
        self.port = port
        self.baudrate = baudrate
        # self.ser = self.init_serial()
        print("serial initialized - waiting for eCAL... ")        
        # init ecal
        self.init_ecal()
        print("eCAL initialized too !")

    def init_serial(self) -> serial.Serial:
        """Initialize the serial port"""
        ser = serial.Serial(self.port, self.baudrate)
        ser.flushInput()
        return ser
    
    def init_ecal(self):
        """Initialize eCAL"""
        ecal_core.initialize(sys.argv, "robot_driver")
        
        # Create the subscribers & set the callbacks
        ProtoSubscriber("set_position", robot_pb.Position).set_callback(self.on_set_position)
        ProtoSubscriber("reach_position", robot_pb.Position).set_callback(self.on_reach_position)
        StringSubscriber("stop_cons").set_callback(self.on_stop_cons)
        ProtoSubscriber("set_pince", robot_pb.SetState).set_callback(self.on_pince)
        ProtoSubscriber("pick_disk", robot_pb.SetState).set_callback(self.on_pick_disk)
        ProtoSubscriber("drop_disk", robot_pb.SetState).set_callback(self.on_drop_disk)
        ProtoSubscriber("set_turbine", robot_pb.SetState).set_callback(self.on_turbine)
        ProtoSubscriber("set_toboggan", robot_pb.SetState).set_callback(self.on_toboggan)
        ProtoSubscriber("set_score", robot_pb.SetState).set_callback(self.on_score)
        ProtoSubscriber("end_match", robot_pb.Bool).set_callback(self.on_end_match)
    
        time.sleep(1.0) # to make time for all to initialize https://github.com/eclipse-ecal/ecal/issues/1012#issuecomment-1467951206



    def read_serial(self) -> str:
        """Read the serial port"""
        return self.ser.readline().decode("utf-8")
    


    def on_set_position(self, topic_name, msg: robot_pb.Position, timestamp):
        """Callback for set_position"""

        print(msg)
        conversion = rb.set_position_bytes(msg.x, msg.y, msg.theta)
        print("conversion", conversion)
        # self.ser.write(conversion)

    def on_reach_position(msg: robot_pb.Position):
        """Callback for reach_position"""
        print("reach_position", msg)

    def on_stop_cons(msg: str):
        """Callback for stop_cons"""
        print("stop_cons", msg)

    def on_pince(msg: robot_pb.SetState):
        """Callback for pince"""
        print("pince", msg)

    def on_pick_disk(msg: robot_pb.SetState):
        """Callback for pick_disk"""
        print("pick_disk", msg)

    def on_drop_disk(msg: robot_pb.SetState):
        """Callback for drop_disk"""
        print("drop_disk", msg)

    def on_turbine(msg: robot_pb.SetState):
        """Callback for turbine"""
        print("turbine", msg)

    def on_toboggan(msg: robot_pb.SetState):
        """Callback for toboggan"""
        print("toboggan", msg)

    def on_score(msg: robot_pb.SetState):
        """Callback for score"""
        print("score", msg)

    def on_end_match(msg: robot_pb.Bool):
        """Callback for end_match"""
        print("end_match", msg)

    
if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "robot_driver")

    driver = Robot_Driver()

    while ecal_core.ok():
        time.sleep(0.01)
        # print(driver.read_serial())

    ecal_core.finalize()

