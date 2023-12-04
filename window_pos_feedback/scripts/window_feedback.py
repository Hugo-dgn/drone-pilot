#! /usr/bin/env python3

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from droneload_msgs.msg import Path as msg_Path
from droneload_msgs.msg import Rectangle3D
from geometry_msgs.msg import PoseStamped
import numpy as np
    
class Path:
    path = None
    
class Dynamics:
    x = 0
    y = 0
    z = 0

class Rectangle:
    corners = None
    
def update_path(msg):
    Path.path = np.array([[point.x, point.y, point.z] for point in msg.points])

def main_window_update(rectangle3D):
    corners = np.array([[rectangle3D.p1.x, rectangle3D.p1.y, rectangle3D.p1.z],
                        [rectangle3D.p2.x, rectangle3D.p2.y, rectangle3D.p2.z],
                        [rectangle3D.p3.x, rectangle3D.p3.y, rectangle3D.p3.z],
                        [rectangle3D.p4.x, rectangle3D.p4.y, rectangle3D.p4.z]])
    Rectangle.corners = np.column_stack([corners[0, :], corners[1,:], corners[2,:], corners[3,:], corners[0,:]])

def update_local_position(msg):
    Dynamics.x = msg.pose.position.x
    Dynamics.y = msg.pose.position.y
    Dynamics.z = msg.pose.position.z

if __name__ == "__main__":
    pub_window_path = rospy.get_param("/droneload/parameters/pub_window_path")
    pub_drone_local_pose = rospy.get_param("/droneload/parameters/pub_drone_local_pose")
    pub_rectangle_window = rospy.get_param("/droneload/parameters/pub_rectangle_window")
    
    rospy.init_node("window_feedback")
    rospy.loginfo("Node window_feedback started")
    
    rospy.Subscriber(pub_window_path, msg_Path, update_path)
    rospy.Subscriber(pub_drone_local_pose, PoseStamped, update_local_position)
    rospy.Subscriber(pub_rectangle_window, Rectangle3D, main_window_update)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    rounding = 1
    lenght = 5
    
    def animate(i):
        ax.clear()
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_xlim([-lenght/2 + rounding*int(Dynamics.x/rounding), lenght/2 + rounding*int(Dynamics.x/rounding)])
        ax.set_ylim([-lenght/2 + rounding*int(Dynamics.y/rounding), lenght/2 + rounding*int(Dynamics.y/rounding)])
        ax.set_zlim([0 + rounding*int(Dynamics.z/rounding), lenght + rounding*int(Dynamics.z/rounding)])
        
        ax.scatter([Dynamics.x], [Dynamics.y], [Dynamics.z], c='r', marker='o')
        
        if Path.path is not None:           
            x, y, z = Path.path.T[0,:], Path.path.T[1,:], Path.path.T[2,:]
            ax.plot(x, y, z)
        if Rectangle.corners is not None:
            ax.plot3D(Rectangle.corners[0,:], Rectangle.corners[1,:], Rectangle.corners[2,:], 'green')
        
        return []
    

    ani = FuncAnimation(fig, animate, frames=range(10), interval=100, repeat=True, blit=True)
    plt.show()