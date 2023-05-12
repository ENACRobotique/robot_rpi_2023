import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import time, os, sys
from math import radians
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_data

lidar_sub = ProtoSubscriber("lidar_data", lidar_data.Lidar)
slow_pub = ProtoPublisher("slow", robot_pb.no_args_func_)
stop_pub = ProtoPublisher("stop", robot_pb.no_args_func_)
resume_pub = ProtoPublisher("resume", robot_pb.no_args_func_)

MIN_RADIUS = 5.0e-2     # Filter points on robot
SLOW_RADIUS = 60.0e-2   # Warning radius: start sending slow messages
STOP_RADIUS = 40.0e-2   # Panic radius: immediately send stop messages


def compute_distances(nb_pts, angle_increment, angles, distances):
    print('message received')


if __name__ == '__main__':
    print('Starting ultimate-fallback collision avoidance')
    ecal_core.initialize(sys.argv, "lidar_evitement")

    lidar_sub.set_callback(compute_distances)

    while ecal_core.ok():
        time.sleep(0.01)

    ecal_core.finalize()

