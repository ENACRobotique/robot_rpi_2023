import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import time, os, sys

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.robot_state_pb2 as robot_pb
from position_fusion.position_smooth import Smoother

DEBUG = True

pos_smoother = Smoother(2000.0, 10, 3) # set timestamp to really high for when replaying
max_x_deviation = 0.1
max_y_deviation = 0.1
max_theta_deviation = 20 #degrees

ecal_core.initialize([], "position_fusion")

sub_lidar = ProtoSubscriber("lidar_pos", robot_pb.Position)
pub_pos = ProtoPublisher("smooth_pos", robot_pb.Position)

def logger(msg):
    if DEBUG:
        ecal_core.log_setlevel(1)
        ecal_core.log_message(str(msg))

def on_lidar_pos(topic_name, lidar_msg , time):
    initial_smooth_pos = pos_smoother.calc_smooth()
    if initial_smooth_pos:
        if abs(initial_smooth_pos[0] - lidar_msg.x) > max_x_deviation:
            logger("x deviation too large for position smoother")
            pos_smoother.flush_data()
        if abs(initial_smooth_pos[1] - lidar_msg.y) > max_y_deviation:
            logger("y deviation too large for position smoothe")
            pos_smoother.flush_data()
        if abs(initial_smooth_pos[2] - lidar_msg.theta) > max_theta_deviation:
            logger("theta deviation too large for position smoothe")
            pos_smoother.flush_data()
    pos_smoother.add_data(lidar_msg.x, lidar_msg.y, lidar_msg.theta, time)
    smooth_pos = pos_smoother.calc_smooth()
    if smooth_pos:
        pub_pos.send(robot_pb.Position(x=smooth_pos[0], y=smooth_pos[1], theta=smooth_pos[2]), time=time)

if __name__ == "__main__":
    
    sub_lidar.set_callback(on_lidar_pos)

    while ecal_core.ok():
        time.sleep(0.5)

    ecal_core.finalize()