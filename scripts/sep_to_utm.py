#!/usr/bin/env python

import roslib; roslib.load_manifest('septentrio')

import rospy
from visualization_msgs.msg import Marker
from septentrio.msg import PvtCartesianMsg

# from math import pow, degrees, radians
# from scipy import mat, cos, sin, arctan, arcsin, sqrt, pi, arctan2, deg2rad, rad2deg

global cur_quat,cur_pos

def callback(msg):
    marker_msg = Marker()
    
    # print marker_msg.markers
    marker_msg.header.frame_id = "/utm"
    marker_msg.header.stamp = msg.header.stamp

    # e,n,u = wgsxyz2utm(msg.x_position,msg.y_position,msg.z_position)

    # marker_msg.pose.position.x = x
    # marker_msg.pose.position.y = y
    # marker_msg.pose.position.z = z
    
    # marker_msg.pose.orientation.x = x
    # marker_msg.pose.orientation.y = y
    # marker_msg.pose.orientation.z = z

    # marker_msg.scale.x = 0.5
    # marker_msg.scale.x = 0.1
    # marker_msg.scale.x = 0.1
    
    # marker_msg.color.r = 0.0
    # marker_msg.color.g = 1.0
    # marker_msg.color.b = 1.0
    # marker_msg.color.a = 1.0

    marker_pub.publish(marker_msg)
    
def wgsxyz2utm(x,y,z):


    return e,n,u

def main():
    global marker_pub
    rospy.init_node('sep_to_utm')
    
    marker_pub = rospy.Publisher("/septentrio/utm_marker", Marker,queue_size=100)
    rospy.Subscriber("/septentrio/pvt_cartesian", PvtCartesianMsg, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass