#! /usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge, CvBridgeError

from droneload_msgs.msg import Rectangle2D, Rectangles2D
from sensor_msgs.msg import Image
from droneload_msgs.msg import Path as msg_Path

import droneload
import cv2
import numpy as np

class Rectangles:
    main = None
    all_rectangle = []

def set_main_rectangle(msg):
    if msg.p1.x == msg.p1.y == msg.p4.x == msg.p4.y == 0:
        Rectangles.main = None
    else:
        corners = [[msg.p1.x, msg.p1.y], [msg.p2.x, msg.p2.y], [msg.p3.x, msg.p3.y], [msg.p4.x, msg.p4.y]]
        main_rect = droneload.rectFinder.Rect(corners)
        main_rect.id = msg.id
        Rectangles.main = main_rect

def set_all_rectangle(msg):
    Rectangles.all_rectangle = []
    for rect_msg in msg.rectangles:
        corners = [[rect_msg.p1.x, rect_msg.p1.y], [rect_msg.p2.x, rect_msg.p2.y], [rect_msg.p3.x, rect_msg.p3.y], [rect_msg.p4.x, rect_msg.p4.y]]
        rect = droneload.rectFinder.Rect(corners)
        rect.id = rect_msg.id
        Rectangles.all_rectangle.append(rect)

def display(camera_msg):
    try:
        # Convertir l'image ROS en image OpenCV
        image = bridge.imgmsg_to_cv2(camera_msg, "mono8")
        
        droneload.rectFinder.draw_rectangles(image, Rectangles.all_rectangle)
        if Rectangles.main is not None:
            droneload.rectFinder.draw_main_rectangle(image, Rectangles.main)

        cv2.imshow("Camera feedback", image)
        cv2.waitKey(100)
            
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    
    pub_front_camera = rospy.get_param("/droneload/parameters/pub_front_camera")
    pub_main_rectangle = rospy.get_param("/droneload/parameters/pub_main_rectangle")
    pub_all_rectangle = rospy.get_param("/droneload/parameters/pub_all_rectangle")
    
    rospy.init_node("camera_feedback")
    rospy.loginfo("Node camera_feedback started")
    
    bridge = CvBridge()
    
    rospy.Subscriber(pub_front_camera, Image, display)
    rospy.Subscriber(pub_main_rectangle, Rectangle2D, set_main_rectangle)
    rospy.Subscriber(pub_all_rectangle, Rectangles2D, set_all_rectangle)
    
    rospy.spin()