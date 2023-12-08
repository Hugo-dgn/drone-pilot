#! /usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError

from droneload_msgs.msg import Point2D, Rectangle2D, Rectangles2D
from sensor_msgs.msg import Image

import droneload
import cv2

class Data:
    main_rect = None

def get_msg_from_rect(rect):
    msg = Rectangle2D()
    p1 = Point2D()
    p2 = Point2D()
    p3 = Point2D()
    p4 = Point2D()

    p1.x = rect.corners2D[0][0]
    p1.y = rect.corners2D[0][1]
    p2.x = rect.corners2D[1][0]
    p2.y = rect.corners2D[1][1]
    p3.x = rect.corners2D[2][0]
    p3.y = rect.corners2D[2][1]
    p4.x = rect.corners2D[3][0]
    p4.y = rect.corners2D[3][1]
    
    msg.p1 = p1
    msg.p2 = p2
    msg.p3 = p3
    msg.p4 = p4
    
    msg.id = rect.id
    
    return msg

def process_image(camera_msg):
    try:
        # Convertir l'image ROS en image OpenCV
        image = bridge.imgmsg_to_cv2(camera_msg, "mono8")

        contours = droneload.rectFinder.get_contours_canny(image, seuil=front_rect_seuil, kernel_size=front_rect_ksize)
        rminLineLength = 1/front_rect_houghlength
        rmaxLineGap = 1/front_rect_houghgap
        threshold = front_rect_houghthreshold
        lines = droneload.rectFinder.get_lines(contours, rminLineLength, rmaxLineGap, threshold)
        rects = droneload.rectFinder.find_rectangles(lines, tol=front_rect_tol)
        
        droneload.rectFinder.remove_old_rects(front_rect_max_lifetime)

        rects_msg = Rectangles2D()
        droneload.rectFinder.fit(rects, front_rect_fit)
        
        for rect in droneload.rectFinder.get_current_rects():
            rects_msg.rectangles.append(get_msg_from_rect(rect))
        all_pub.publish(rects_msg)
        
        window_id = rospy.get_param("/droneload/parameters/window_id")
        if window_id == -1:
            rect = droneload.rectFinder.get_main_rect(front_rect_min_fit_success)
            Data.main_rect = rect
        else:
            for rect in droneload.rectFinder.get_current_rects():
                if rect.id == window_id:
                    Data.main_rect = rect
                    break
            else:
                Data.main_rect = None
        
        if Data.main_rect is not None:
            main_msg = get_msg_from_rect(Data.main_rect)
            
            main_pub.publish(main_msg)
        
        else:
            main_pub.publish(default_msg)
            
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    
    path_front_camera_calibration = rospy.get_param("/droneload/parameters/path_front_camera_calibration")
    front_image_size = rospy.get_param("/droneload/parameters/front_image_size")
    
    pub_main_rectangle = rospy.get_param("/droneload/parameters/pub_main_rectangle")
    pub_all_rectangles = rospy.get_param("/droneload/parameters/pub_all_rectangle")
    pub_front_camera = rospy.get_param("/droneload/parameters/pub_front_camera")
    
    front_rect_tol = rospy.get_param("/droneload/parameters/front_rect_tol")
    front_rect_fit = rospy.get_param("/droneload/parameters/front_rect_fit")
    front_rect_max_lifetime = rospy.get_param("/droneload/parameters/front_rect_max_lifetime")
    front_rect_min_fit_success = rospy.get_param("/droneload/parameters/front_rect_min_fit_success")
    front_rect_seuil = rospy.get_param("/droneload/parameters/front_rect_seuil")
    front_rect_ksize = rospy.get_param("/droneload/parameters/front_rect_ksize")
    front_rect_houghlength = rospy.get_param("/droneload/parameters/front_rect_houghlength")
    front_rect_houghgap = rospy.get_param("/droneload/parameters/front_rect_houghgap")
    front_rect_houghthreshold = rospy.get_param("/droneload/parameters/front_rect_houghthreshold")
    
    droneload.rectFinder.calibration(path_front_camera_calibration)
    droneload.rectFinder.calibrate_image_size(front_image_size)
    
    rospy.init_node("rectangles")
    rospy.loginfo("Node rectangles started")
    
    main_pub = rospy.Publisher(pub_main_rectangle, Rectangle2D, queue_size=1)
    all_pub = rospy.Publisher(pub_all_rectangles, Rectangles2D, queue_size=1)
    
    bridge = CvBridge()
    
    default_msg = Rectangle2D()
    default_msg.p1.x = 0
    default_msg.p1.y = 0
    default_msg.p2.x = 0
    default_msg.p2.y = 0
    default_msg.p3.x = 0
    default_msg.p3.y = 0
    default_msg.p4.x = 0
    default_msg.p4.y = 0
    
    rospy.Subscriber(pub_front_camera, Image, process_image)
    
    rospy.spin()