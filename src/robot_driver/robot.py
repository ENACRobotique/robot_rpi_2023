import comm

class robot:
    def __init__(self,x=0,y=0,theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.pos = (self.x,self.y,self.theta)
        self.radio = comm.Radio()
        self.last_target_x = x
        self.last_target_y = y
        self.last_target_theta = theta
        self.xy_accuracy = 0.1  #m
        self.theta_accuracy = 0.1 # a changer

    def __repr__(self) -> str:
        return str(self.pos)

    def get_position(self):
        self.x = self.radio.data_robot["x"]
        self.y = self.radio.data_robot["y"]
        self.theta = self.radio.data_robot["theta"]
        return  self.pos
    
    def set_target_position(self,x=float,y=float,theta=float)->None:
        self.radio.setTargetPosition(x,y,theta)

    def is_target_reach(self)-> bool:

        reach = 
        return reach
    



    
