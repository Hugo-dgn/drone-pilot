#! /usr/bin/env python3

import rospy
import numpy as np
import math
from drone import Drone

from mavros_msgs.msg import State
from droneload_msgs.msg import Path as msg_Path
from droneload_msgs.msg import Rectangle3D
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from tf.transformations import quaternion_matrix


class Data:
    window_path = []


def update_window_path(msg):
    Data.window_path = np.array(msg.points)

def transition(copter):
    if copter.flight_state == 1 and copter.state.mode != "OFFBOARD":
        copter.offboard()
    
if __name__ == "__main__":
    
    main_loop_rate = rospy.get_param("/droneload/parameters/main_loop_rate")
    pub_window_path = rospy.get_param("/droneload/parameters/pub_window_path")
    
    rospy.Subscriber(pub_window_path, msg_Path, update_window_path)

    
    rospy.init_node("pilot")
    rate = rospy.Rate(main_loop_rate)
    
    copter = Drone(rate)
    
    while not rospy.is_shutdown():
        copter.flight_state = rospy.get_param("/droneload/parameters/drone_state")
        transition(copter)
        
        if copter.flight_state == 1:
            copter.follow(Data.window_path)
        rate.sleep()