# library to smooth position using moving average

from dataclasses import dataclass
from typing import List
from queue import Queue


@dataclass
class Smoother:
    max_timeframe: float =  2.0 # in seconds, time when it will forget the oldest data
    max_data: int = 10 # max number of data to store
    min_data: int = 3 # min number of data to store
    x_data: List[float] = None
    y_data: List[float] = None
    theta_data: List[float] = None
    timestamp_data: List[float] = None

    def __post_init__(self):
        self.flush_data()

    def add_data(self, x, y, theta, timestamp):
        # add data to the buffer & remove too old data
        # flush it before if the robot has moved !
        self.x_data.append(x)
        self.y_data.append(y)
        self.theta_data.append(theta)
        self.timestamp_data.append(timestamp)
        
        if len(self.x_data) > self.max_data:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.theta_data.pop(0)
            self.timestamp_data.pop(0)

        while self.timestamp_data[0] < self.timestamp_data[-1] - self.max_timeframe * 1000000: # removes too old data #converts second to microsecond
            print("removing old data")
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.theta_data.pop(0)
            self.timestamp_data.pop(0)
        
    def calc_smooth(self):
        # calculate smooth position using moving average
        if len(self.x_data) < self.min_data:
            return None
        # x = sum(self.x_data) / len(self.x_data)
        # y = sum(self.y_data) / len(self.y_data)
        # theta = sum(self.theta_data) / len(self.theta_data)
        x = sorted(self.x_data)[round((len(self.x_data)+1)/2)]
        y = sorted(self.y_data)[round((len(self.x_data)+1)/2)]
        theta = sorted(self.theta_data)[round((1+len(self.theta_data))/2)]
        return (x, y, theta)

    def flush_data(self):
        #flush data
        self.x_data = []
        self.y_data = []
        self.theta_data = []
        self.timestamp_data = []

