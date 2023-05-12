# ecal boring things:

sub_position = ProtoSubscriber("set_position", robot_pb.Position)

def on_robot_dest(topic_name, travel_msg, time):
    global last_known_dest
    last_known_dest = (travel_msg.x, travel_msg.y, travel_msg.theta)