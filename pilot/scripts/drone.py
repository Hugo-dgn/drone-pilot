import numpy as np

import rospy

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class Drone:
    
    def __init__(self, rate):
        self.flight_state = rospy.get_param("/droneload/parameters/drone_state")
        self.state = None
        self.poseStamped = None
        self.rate = rate
        
        pub_drone_state = rospy.get_param("/droneload/parameters/pub_drone_state")
        pub_drone_local_pose_set_point = rospy.get_param("/droneload/parameters/pub_drone_local_pose_set_point")
        pub_drone_local_pose = rospy.get_param("/droneload/parameters/pub_drone_local_pose")
        
        service_drone_arming = rospy.get_param("/droneload/parameters/service_drone_arming")
        service_drone_set_mode = rospy.get_param("/droneload/parameters/service_drone_set_mode")
        
        rospy.Subscriber(pub_drone_state, State, self.update_state)
        rospy.Subscriber(pub_drone_local_pose, PoseStamped, self.update_pose)
        
        self.local_pos_pub = rospy.Publisher(pub_drone_local_pose_set_point, PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy(service_drone_arming, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(service_drone_set_mode, SetMode)
        
        self.reposition_threeshold = rospy.get_param("/droneload/parameters/reposition_threeshold")
        self.window_min_dist_next_point = rospy.get_param("/droneload/parameters/window_min_dist_next_point")
    
    def update_state(self, state):
        self.state = state
    
    def update_pose(self, poseStamped):
        self.poseStamped = poseStamped
    
    def offboard(self):
        while self.state is None or self.poseStamped is None:
            self.rate.sleep()
        while(not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()

        for _ in range(100):
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.poseStamped)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = "OFFBOARD"

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        while not rospy.is_shutdown() and (not self.state.armed or not self.state.mode == "OFFBOARD"):
            if self.state.mode != "OFFBOARD":
                self.set_mode_client.call(offb_set_mode).mode_sent
            else:
                if not self.state.armed:
                    self.arming_client.call(arm_cmd)
            
            self.local_pos_pub.publish(self.poseStamped)
            self.rate.sleep()

        rospy.loginfo("Takeoff successful")
    
    def follow(self, path):
        if len(path) == 0:
            return False
        current_point = np.array([self.poseStamped.pose.position.x, self.poseStamped.pose.position.y, self.poseStamped.pose.position.z]).reshape(1, 3)
        path_array = np.array([[p.x, p.y, p.z] for p in path])
        
        dist_list = np.linalg.norm(path_array - current_point, axis=1)
        
        current_path_point_index = np.argmin(dist_list)
        current_path_point = path[current_path_point_index]
        
        if dist_list[current_path_point_index] > self.reposition_threeshold:
            self.poseStamped.pose.position.x = current_path_point.x
            self.poseStamped.pose.position.y = current_path_point.y
            self.poseStamped.pose.position.z = current_path_point.z
        elif len(path) > current_path_point_index+1:
            for i, l in enumerate(dist_list[current_path_point_index+1:]):
                if l > self.window_min_dist_next_point:
                    break
            next_path_point = path[current_path_point_index+1+i]
            self.poseStamped.pose.position.x = next_path_point.x
            self.poseStamped.pose.position.y = next_path_point.y
            self.poseStamped.pose.position.z = next_path_point.z 
        
        self.local_pos_pub.publish(self.poseStamped)
        
        return True