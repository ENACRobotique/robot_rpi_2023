import struct
import serial
import threading
from time import time, gmtime, sleep
from enum import Enum

def temps_deb (timestamp):
    """Input : t (float) : value given by time()
    Output : a formated string that gives a more explicit time than t
    !!! Cette fonction est à l'heure d'été."""
    itm = gmtime(timestamp+2*3600)
    return '{:04d}/{:02d}/{:02d}\t{:02d}:{:02d}:{:02d}'.format (itm.tm_year, itm.tm_mon,
            itm.tm_mday, itm.tm_hour, itm.tm_min, itm.tm_sec) +'{:.3}'.format (timestamp%1)[1:]
class radioStates(Enum):
    WAITING_FIRST_BYTE=0
    BETWEEN_START_BYTES=1
    WAITING_TYPE=2
    WAITING_REST_OF_NORMAL_MESSAGE=3
    WAITING_REST_OF_STRING_MESSAGE=4

class messageARecevoir(Enum):
    REPORT_POSITION="p"
    REPORT_VITESSE="v"
    DEBUT_MATCH="T"
    CONFIRMATION_ACTION="d"

    MESSAGE_POUR_LOG="M"

PORT_NAME = "/dev/ttyACM1"
class Radio:
    def __init__(self):
        self.PROTOCOL_VERSION =1
        self.continueListening = False
        self.serialObject = serial.Serial (port = PORT_NAME, baudrate=115200, timeout =1)
        self.radioState = radioStates.WAITING_FIRST_BYTE
        self.listeningThread = threading.Thread(target=self.listen)

    def startListening (self):
        self.continueListening =True
        self.listeningThread.start()

    def stopListening (self):
        self.listen = False

    def __repr__(self):
        return "Radio haut niveau"
    
    def verifyAndExec(self,byteArray,typeReçu):
        match typeReçu:
            case messageARecevoir.REPORT_POSITION:
                (x,y,theta,chksum)=struct.unpack("fffB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('p')
                for byte in byteArray[:-1]:
                    sum+=byte
                if sum%256 == chksum:
                    self.on_odom_position(x,y,theta)
                    print ("success : Pos ({}, {}, {})\n\n".format(x,y,theta))
                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'p'+byteArray)
            case messageARecevoir.REPORT_VITESSE:
                (Vx,Vy,Vtheta,chksum)=struct.unpack("fffB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('v')
                for byte in byteArray[:-1]:
                    sum+=byte
                if sum%256 == chksum:
                    print ("success : Speed ({}, {}, {})\n\n".format(Vx,Vy,Vtheta))
                    #TODO do something with message reception
                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'v'+byteArray)
            case messageARecevoir.DEBUT_MATCH:
                (chksum,)=struct.unpack("B",byteArray)
                if chksum == ord('T')+self.PROTOCOL_VERSION:
                    print ("success : MATCH STARTED")
                    #TODO do something with message reception
                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'T'+byteArray)
            case messageARecevoir.CONFIRMATION_ACTION:
                (num,chksum)=struct.unpack("BB",byteArray)
                sum = self.PROTOCOL_VERSION + ord('d') +byteArray[0]
                if sum%256 == chksum:
                    print ("success : Action Confirmed : {}\n\n".format(num))
                    #TODO do something with message reception
                else:
                    print("FAILED CHECKSUM : MessageError")
                    print(b'd'+byteArray)

    def on_odom_position(self, x, y, theta):
        raise NotImplementedError()


    def listen(self):
        print("Starting Listening")
        numberOfExpectedBytes =0
        typeReçu =None
        stringMessage =""
        c=None
        while self.continueListening:
            sleep(0.0002)#pour éviter de bloquer le processeur
            match self.radioState:
                case radioStates.WAITING_FIRST_BYTE:
                    while (c!='\n') and (self.serialObject.in_waiting !=0):
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                    if c=='\n':
                        self.radioState=radioStates.BETWEEN_START_BYTES
                case radioStates.BETWEEN_START_BYTES:
                    if self.serialObject.in_waiting !=0:
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                        self.radioState = radioStates.WAITING_TYPE if c=='\n' else radioStates.WAITING_FIRST_BYTE
                case radioStates.WAITING_TYPE:
                    if self.serialObject.in_waiting !=0:
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                        match c:
                            case messageARecevoir.REPORT_POSITION.value:
                                numberOfExpectedBytes=13#3 floats(4o) + 1o checksum
                                typeReçu = messageARecevoir.REPORT_POSITION
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.REPORT_VITESSE.value:
                                numberOfExpectedBytes=13#3 floats(4o) + 1o checksum
                                typeReçu = messageARecevoir.REPORT_VITESSE
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.DEBUT_MATCH.value:
                                numberOfExpectedBytes=1#1o checksum
                                typeReçu = messageARecevoir.DEBUT_MATCH
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.CONFIRMATION_ACTION.value:
                                numberOfExpectedBytes=2#1o numAction + 1o checksum
                                typeReçu = messageARecevoir.CONFIRMATION_ACTION
                                self.radioState=radioStates.WAITING_REST_OF_NORMAL_MESSAGE
                            case messageARecevoir.MESSAGE_POUR_LOG.value:
                                self.radioState=radioStates.WAITING_REST_OF_STRING_MESSAGE
                                stringMessage=""

                            case '\n':
                                pass
                            case _:
                                self.radioState=radioStates.WAITING_FIRST_BYTE
                case radioStates.WAITING_REST_OF_NORMAL_MESSAGE:
                    if self.serialObject.in_waiting >= numberOfExpectedBytes:
                        self.verifyAndExec(self.serialObject.read(numberOfExpectedBytes),typeReçu)
                        self.radioState=radioStates.WAITING_FIRST_BYTE
                    
                case radioStates.WAITING_REST_OF_STRING_MESSAGE:
                    while (c!='\n') and (self.serialObject.in_waiting !=0):
                        c=chr(struct.unpack("B",self.serialObject.read(1))[0])
                        stringMessage+=c
                    if (c=='\n'):
                        tStampString = temps_deb(time())
                        print(tStampString+"\t"+stringMessage)
                        #TODO : store that print in a log file
                        self.radioState=radioStates.WAITING_FIRST_BYTE
if __name__=="__main__":
    radio=Radio()
    radio.startListening()
    while True:
        pass