import struct
from enum import Enum
from serial import Serial

import time


class Cloud:
    def __init__(self):
        self.count = 0
        self.points = []
        self.max_angle = float('-inf')
        self.min_angle = float('inf')
        self.distances = []
        self.angles = []
    
    def add(self, distance, angle):
        if self.filter(distance):
            self.count += 1
            self.points.append((distance, angle))
            self.max_angle = max(self.max_angle, angle)
            self.min_angle = min(self.min_angle, angle)
            self.distances.append(distance)
            self.angles.append(angle)

    def reverse(self): 
        self.distances.reverse()
        self.angles.reverse()
        # self.min_angle = self.angles[0]
        # self.max_angle = self.angles[1]

    def filter(self, distance):
        # Return True if the point should be kept, False otherwise
        return True

    def span(self):
        return self.max_angle - self.min_angle


# This enum contains the part of the lidar message that is expected by the driver
class Part(Enum):
    # The driver is waiting for the start byte `0x54`
    START = 0 
    # The driver is waiting for the length byte
    LEN = 1
    # The driver is waiting for the message data
    DATA = 2


class Driver:
    def __init__(self, on_scan = lambda distance, theta: None):
        # Driver settings and state data
        self.serial_port = 'COM4' # TODO: Get serial port from settings
        self.serial = Serial(port=self.serial_port, 
                            baudrate=230400,
                            bytesize=8,
                            parity='N',
                            stopbits=1)
        self.expected_type = Part.START
        self.expected_length = 1
        # Current message information
        self.count = 0 # Number of point samples in the message
        # Point cloud
        self.total_angle = 0
        self.cloud = Cloud()
        self.on_scan = on_scan

    def scan(self):
        while True:
            # Getting data from the serial port
            data = self.serial.read(self.expected_length)

            if self.expected_type == Part.START:
                # Check if we have received the start byte
                if data[0] == 0x54:
                    self.expected_type = Part.LEN
                    self.expected_length = 1

            elif self.expected_type == Part.LEN:
                # Decode data length message
                self.count = data[0] & 0x0E
                self.expected_type = Part.DATA
                self.expected_length = 9 + 3 * self.count

            elif self.expected_type == Part.DATA:
                # Extract lidar state data
                speed, = struct.unpack('<H', data[0:2])
                start_angle = struct.unpack('<H', data[2:4])[0] / 100
                
                data_end = self.expected_length - 5
                message_data = data[4:data_end]
                
                end_angle = struct.unpack('<H', data[data_end:data_end+2])[0] / 100
                timestamp = struct.unpack('<H', data[data_end+2:data_end+4])[0]
                crc = struct.unpack('<B', data[data_end+4:])

                # Extract point cloud
                if end_angle < start_angle:
                    step = (end_angle + 360 - start_angle) / (self.count -1)
                else:
                    step = (end_angle - start_angle) / (self.count -1)

                self.total_angle += self.count * step

                for i in range(self.count):
                    distance = struct.unpack('<H', message_data[3*i:3*i+2])[0]
                    quality = struct.unpack('<B', message_data[3*i+2:3*i+3])[0]
                    angle_clkwise = start_angle + i * step

                    if angle_clkwise >= 360:
                        angle_clkwise -= 360

                    # conversion from clockwise to anticlockwise/trigonometric angle
                    angle_anticlkwise = 360 - angle_clkwise
                    distance_meter = distance / 1000 # conversion mm to m
                    self.cloud.add(distance_meter, angle_anticlkwise)

                # conversion from clockwise to anticlockwise/trigonometric angle
                self.cloud.reverse()

                if self.total_angle >= 360:
                    # The points are back to the beginning
                    print('count:', self.cloud.count, ', span:', self.cloud.span())
                    self.on_scan(self.cloud.angles, self.cloud.distances)
                    # We can start a new point cloud
                    self.total_angle = 0
                    self.cloud = Cloud()

                self.expected_type = Part.START
                self.expected_length = 1


if __name__ == '__main__':
    driver = Driver()
    driver.scan()