from dataclasses import dataclass
from typing import List
from struct import pack, unpack

protocolVersion = 1 # used for chksum calculation
    
#### Date Emitters (rpi>stm) ####

def set_position_bytes(x: float, y: float, theta: float) -> bytes:
    # "\n\np[x][y][theta][chksum]"
    msg = pack(">cfff", b"p", x, y, theta)
    chksum = (sum([x for x in msg]) + protocolVersion) % 256
    return b'\n' + b'\n' + msg + pack(">B", chksum)



# "\n\nr[x][y][theta][chksum]"
# "\n\nS[chksum]"
# "\n\ns[chksum]"
# "\n\nR[chksum]"
# "\n\ng[Pos][chksum]"
# "\n\na[Pos][num][chksum]"
# "\n\nd[Pos][num][chksum]"
# "\n\nt[etat][chksum]"
# "\n\nD[chksum]"
# "\n\nt[etat][chksum]"
# "\n\nP[score][chksum]"
# "\n\nM[string]\n"

##### Data Receivers (stm>rpi) #####

@dataclass
class DataReceiver:
    first_letter: str

    def parse_bytes(self, string: str) -> List:
        # parse the string and return a protobuf message
        raise NotImplementedError()

class ReceivePos(DataReceiver):
    def __init__():
        super.first_letter = "p"

    def parse_bytes(self, bytes: str) -> List:
        # "\n\np[x][y][theta][chksum]"
        _, x, y, theta, chksum = unpack(">cfffB", bytes) 
        calc_chksum = (sum([x for x in bytes[2:15]]) + protocolVersion) % 256
        if calc_chksum != chksum:
            raise ValueError("Checksum error for position")
        return [x, y, theta]
    
class ReceiveSpeed(DataReceiver):
    def __init__():
        super.first_letter = "v"

    def parse_bytes(self, bytes: str) -> List:
        # "\n\nv[Vx][Vy][Vtheta][chksum]"
        _, x, y, theta, chksum = unpack(">cfffB", bytes) 
        calc_chksum = (sum([x for x in bytes[2:15]]) + protocolVersion) % 256
        if calc_chksum != chksum:
            raise ValueError("Checksum error for speed")
        return [x, y, theta]
    
class ReceiveTirette(DataReceiver):
    def __init__():
        super.first_letter = "T"

    def parse_bytes(self, bytes: str) -> List:
        # "\n\nT[chksum]"
        _, chksum = unpack(">cB", bytes) 
        calc_chksum = bytes[2] + protocolVersion % 256
        if calc_chksum != chksum:
            raise ValueError("Checksum error for tirette")
        return True

class ReceiveConfirm(DataReceiver):
    def __init__():
        super.first_letter = "d"

    def parse_bytes(self, bytes: str) -> List:
        # "\n\nd[num][chksum]"
        _, num, chksum = unpack(">cBB", bytes)
        calc_chksum = (sum([x for x in bytes[2:4]]) + protocolVersion) % 256
        if calc_chksum != chksum:
            raise ValueError("Checksum error for executed action confirmation")
        return num
    
class ReceiveMsg(DataReceiver):
    def __init__():
        super.first_letter = "M"
    
    def parse_bytes(self, string: str) -> List:
        # "\n\nM[string]\n"
        _, string, _ = unpack(">c%dsB", string)
        return string.decode()
    

