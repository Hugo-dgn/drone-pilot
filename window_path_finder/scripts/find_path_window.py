#! /usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_matrix

from droneload_msgs.msg import Rectangle2D, Rectangle3D, Path
from geometry_msgs.msg import TwistStamped, Point
from geometry_msgs.msg import PoseStamped

import droneload
import cv2
import numpy as np

correction_matrix = np.array(
    [[0, 0, 1],
     [-1, 0, 0],
     [0, -1, 0]]
)


target_rect_corners = 0.8/2*np.array(
                            [[-1, 0, -1],
                            [1, 0, -1],
                            [1, 0, 1],
                            [-1, 0, 1]]
                        )

class Dynamics:
    x = 0
    y = 0
    z = 0
    rotation_matrix = np.eye(3)

class Data:
    path = None
    rect3D = None
    rect3D_id = None
    n = None
def get_main_rectangle(msg):
    if msg.p1.x == msg.p1.y == msg.p4.x == msg.p4.y == 0:
        return None
    else:
        corners = [[msg.p1.x, msg.p1.y], [msg.p2.x, msg.p2.y], [msg.p3.x, msg.p3.y], [msg.p4.x, msg.p4.y]]
        main_rect = droneload.rectFinder.Rect(corners)
        main_rect.id = msg.id
        return main_rect

def find_path(main_rect_msg):
    main_rect = get_main_rectangle(main_rect_msg)
    if main_rect is not None:
        main_rect.define_3D(target_rect_corners)
        pos, retval, rvec, tvec = main_rect.compute()
            
        x0 = np.array([Dynamics.x, Dynamics.y, Dynamics.z])
        
        R, _ = cv2.Rodrigues(rvec)
        
        real_rect = Dynamics.rotation_matrix@correction_matrix@(tvec + R@target_rect_corners.T) + x0.reshape(3, 1)
        
        if main_rect.id == Data.rect3D_id:
            Data.rect3D = 1/(Data.n+1)*(real_rect + Data.n*Data.rect3D)
            Data.n = min(Data.n + 1, 20)
        else:
            Data.rect3D = real_rect
            Data.rect3D_id = main_rect.id
            Data.n = 0
        
        real_rect = Data.rect3D
        
        window = droneload.pathFinder.Window(real_rect.T)
        x1 = window.p
        
        n = window.n * np.sign(np.dot(window.n, window.p-x0)) * 2 * np.linalg.norm(x1-x0)
        
        n_index_max = np.argmax(np.abs(n))
        alpha = 2 * abs((x1[n_index_max] - x0[n_index_max]) / n[n_index_max])
        
        n = alpha * n
        
                
        L = 1
        n_point = path_window_points
        
        path = droneload.pathFinder.get_path(x0=x0, x1=x1, n=n, L=L, n_point=n_point)
        Data.path = path.T
        
        msg_path = Path()
        for x, y, z in Data.path:
            msg_path.points.append(Point(x=x, y=y, z=z))
        
        path_pub.publish(msg_path)
        
        main_window_msg = Rectangle3D()
        main_window_msg.p1 = Point(x=window.corners[0][0], y=window.corners[0][1], z=window.corners[0][2])
        main_window_msg.p2 = Point(x=window.corners[1][0], y=window.corners[1][1], z=window.corners[1][2])
        main_window_msg.p3 = Point(x=window.corners[2][0], y=window.corners[2][1], z=window.corners[2][2])
        main_window_msg.p4 = Point(x=window.corners[3][0], y=window.corners[3][1], z=window.corners[3][2])
        main_window_pub.publish(main_window_msg)

def update_local_position(msg):
    quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rotation_matrix = quaternion_matrix(quaternion)
    Dynamics.rotation_matrix = rotation_matrix[0:3, 0:3]
    Dynamics.x = msg.pose.position.x
    Dynamics.y = msg.pose.position.y
    Dynamics.z = msg.pose.position.z
            

if __name__ == "__main__":
    
    rospy.init_node("window_path_finder")
    rospy.loginfo("Node window_path_finder started")
    
    pub_main_rectangle = rospy.get_param("/droneload/parameters/pub_main_rectangle")
    pub_window_path = rospy.get_param("/droneload/parameters/pub_window_path")
    pub_drone_local_pose = rospy.get_param("/droneload/parameters/pub_drone_local_pose")
    pub_rectangle_window = rospy.get_param("/droneload/parameters/pub_rectangle_window")
    
    path_front_camera_calibration = rospy.get_param("/droneload/parameters/path_front_camera_calibration")
    path_window_points = rospy.get_param("/droneload/parameters/path_window_points")

    droneload.rectFinder.calibration(path_front_camera_calibration)
    rospy.Subscriber(pub_main_rectangle, Rectangle2D, find_path)
    rospy.Subscriber(pub_drone_local_pose, PoseStamped, update_local_position)

    
    path_pub = rospy.Publisher(pub_window_path, Path, queue_size=1)
    main_window_pub = rospy.Publisher(pub_rectangle_window, Rectangle3D, queue_size=1)
    
    rospy.spin()