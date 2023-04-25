#!/usr/bin/python3

# Usage: ./natnet2ecal.py -i <id> -f <frequency>
# ./natnet2ecal.py -f 10 -i 128

from __future__ import print_function


import sys
from os import path, getenv
from time import time, sleep
import numpy as np
from pyquaternion import Quaternion as Quat
from collections import deque
import argparse
from math import degrees

# import NatNet client
from NatNetClient import NatNetClient

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher

sys.path.append(path.join(path.dirname(__file__), '../..'))
import generated.robot_state_pb2 as robot_pb

# parse args
parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
)

parser.add_argument('-s', '--server', dest='server', default="127.0.0.1", help="NatNet server IP address")
parser.add_argument('-m', '--multicast_addr', dest='multicast', default="239.255.42.99", help="NatNet server multicast address")
parser.add_argument('-dp', '--data_port', dest='data_port', type=int, default=1511, help="NatNet server data socket UDP port")
parser.add_argument('-cp', '--command_port', dest='command_port', type=int, default=1510, help="NatNet server command socket UDP port")
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
parser.add_argument('-o', '--old_natnet', dest='old_natnet', action='store_true', help="Change the NatNet version to 2.9")
parser.add_argument('-f', '--frequency', type=float, default=10, help="frequency")
parser.add_argument('-i', '--ac_id', type=int, default=10, help="frequency")

args = parser.parse_args()



class OptitrackBridge:
    def __init__(self):
        ecal_core.initialize(sys.argv, "optitrack bridge")
        self.pos_pub = ProtoPublisher('optitrack_pos', robot_pb.Position)
        self.last_time = {}     # {id: time}
        natnet_version = (3,0,0,0)
        if args.old_natnet:
            natnet_version = (2,9,0,0)
        self.natnet = NatNetClient(
                server=args.server,
                rigidBodyListListener=self.receiveRigidBodyList,
                dataPort=args.data_port,
                commandPort=args.command_port,
                verbose=args.verbose,
                version=natnet_version)

    def run(self):
        print("Starting Natnet3.x to ecal %s" % (args.server))
        self.natnet.run()

    def stop(self):
        self.natnet.stop()



    def receiveRigidBodyList(self, rigidBodyList, stamp ):
        for (ac_id, pos, quat, valid) in rigidBodyList:
            if not valid:
                # skip if rigid body is not valid
                continue

            lt = self.last_time.setdefault(ac_id, 0)
            now = time()
            if now-lt > 1/args.frequency:
                q = Quat(quat[3], quat[0], quat[1], quat[2])
                x, y, _ = pos
                yaw = q.yaw_pitch_roll[0]

                print(ac_id, x, y, degrees(yaw))
                self.last_time[ac_id] = now
                if ac_id == args.ac_id:
                    self.pos_pub.send(robot_pb.Position(x=x, y=y, theta=yaw))



if __name__ == "__main__":
    bridge = OptitrackBridge()
    try:
        bridge.run()
        while ecal_core.ok():
            sleep(1)
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down ivy and natnet interfaces...")
        bridge.stop()
    except OSError:
        print("Natnet connection error")
        bridge.stop()
        exit(-1)
