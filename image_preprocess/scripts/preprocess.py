#! /usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

import droneload
import cv2

class Images:
    front = None

def process_image(camera_msg):
    try:
        # Convertir l'image ROS en image OpenCV
        frame = bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        
        frame = droneload.rectFinder.undistort(frame)

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        image_ros = bridge.cv2_to_imgmsg(image, encoding="mono8")
        
        Images.front = image_ros
        
        pub_front.publish(Images.front)

            
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    
    pub_front_camera = rospy.get_param("/droneload/parameters/pub_front_camera")
    pub_front_camera_image_raw = rospy.get_param("/droneload/parameters/pub_front_camera_image_raw")
    
    path_front_camera_calibration = rospy.get_param("/droneload/parameters/path_front_camera_calibration")
    
    droneload.rectFinder.calibration(path_front_camera_calibration)
    bridge = CvBridge()
    
    rospy.init_node("preprocess")
    rospy.loginfo("Node preprocess started")
    pub_front = rospy.Publisher(pub_front_camera, Image, queue_size=1)
    
    rospy.Subscriber(pub_front_camera_image_raw, Image, process_image)
    
    rospy.spin()