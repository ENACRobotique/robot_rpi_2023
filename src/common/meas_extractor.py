#!/usr/bin/python3

from ecal.measurement import hdf5 as h
import generated.robot_state_pb2 as pbr
import sys

DEFAULT_MEAS = "/home/robot/ecal_meas/azertyuiop/giskard/giskard.hdf5"
DEFAULT_CHANNEL = "odom_pos"
DEFAULT_OUT = "pos.csv"

if __name__ == '__main__':
    meas_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MEAS
    channel_name = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_CHANNEL
    outfile = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_OUT

    m=h.Meas()
    m.open(meas_file, 0)

    channels = m.get_channel_names()
    if channel_name not in channels:
        print(f"channel {channel_name} not in {meas_file}!")
        exit(1)

    channel_type = m.get_channel_type('odom_pos').split('.')[-1]

    t0 = m.get_min_timestamp(channel_name)
    with open(outfile, 'w') as fic:
        for entry in m.get_entries_info(channel_name):
            t = (entry['rcv_timestamp'] - t0) / 1e6
            id = entry['id']
            data = m.get_entry_data(id)
            pos = pbr.__getattribute__(channel_type).FromString(data)
            print(f"{t:.2f}")
            print(pos)
            fic.write(f"{t:.2f};{pos.x:.3f};{pos.y:.3f};{pos.theta:.3f}\n")
