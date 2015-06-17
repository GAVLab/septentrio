#!/usr/bin/env python

import roslib; roslib.load_manifest('septentrio')

import rospy
import tf
# from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, PoseStamped

from septentrio.msg import PvtCartesianMsg,AttitudeEulerMsg

from tf.transformations import quaternion_from_euler as qfe

from geodesy import utm
# from math import pow, degrees, radians
# from scipy import mat, cos, sin, arctan, arcsin, sqrt, pi, arctan2, deg2rad, rad2deg
from math import pow
from scipy import sqrt, mat, pi,arctan2,sin,cos

global cur_quat,cur_pos
cur_pos = Point()
cur_quat = qfe(0.1,0.1,0.1)

def att_callback(msg):
    global cur_quat,cur_pos
    cur_quat = qfe(msg.heading,msg.pitch,msg.roll)

    # publish_pose(cur_pos,cur_quat)

def pos_callback(msg):
    global cur_quat,cur_pos
    lat,lon,alt = wgsxyz2lla(msg.x_position,msg.y_position,msg.z_position)
    
    enu = utm.fromLatLong(lat, lon, alt).toPoint()
    cur_pos = enu

    publish_pose(cur_pos,cur_quat)
    
def publish_pose(pos,quat):
    # print pos.x
    pose_msg = PoseStamped()

    pose_msg.header.frame_id = "/utm"
    pose_msg.header.stamp = rospy.Time.now()#msg.header.stamp

    pose_msg.pose.position.x = pos.x
    pose_msg.pose.position.y = pos.y
    pose_msg.pose.position.z = pos.z

    pose_msg.pose.orientation.x = quat[0]
    pose_msg.pose.orientation.y = quat[1]
    pose_msg.pose.orientation.z = quat[2]
    pose_msg.pose.orientation.w = quat[3]

    br = tf.TransformBroadcaster()
    br.sendTransform((pos.x,pos.y, pos.z),
    (quat[0],quat[1],quat[2],quat[3]),
    rospy.Time.now(),'ref_base_footprint','utm')

    # print br
    pose_pub.publish(pose_msg)

def wgsxyz2lla(x,y,z):

    #  This dual-variable iteration seems to be 7 or 8 times faster than
    #  a one-variable (in latitude only) iteration.  AKB 7/17/95 

    A_EARTH = 6378137
    flattening = 1/298.257223563
    NAV_E2 = (2-flattening)*flattening # also e^2
    rad2deg = 180/pi

    wlon = arctan2(y, x)*rad2deg
    

    # Make initial lat and alt guesses based on spherical earth.
    rhosqrd = x*x + y*y
    rho = sqrt(rhosqrd)
    templat = arctan2(z, rho)
    tempalt = sqrt(rhosqrd + z*z) - A_EARTH
    rhoerror = 1000.0
    zerror   = 1000.0

    #  Newton's method iteration on templat and tempalt makes
    #   the residuals on rho and z progressively smaller.  Loop
    #   is implemented as a 'while' instead of a 'do' to simplify
    #   porting to MATLAB

    while ((abs(rhoerror) > 1e-6) | (abs(zerror) > 1e-6)):
        slat = sin(templat)
        clat = cos(templat)
        q = 1 - NAV_E2*slat*slat
        r_n = A_EARTH/sqrt(q)
        drdl = r_n*NAV_E2*slat*clat/q # d(r_n)/d(latitutde) 

        rhoerror = (r_n + tempalt)*clat - rho
        zerror   = (r_n*(1 - NAV_E2) + tempalt)*slat - z

        #             --                               -- --      --
        #             |  drhoerror/dlat  drhoerror/dalt | |  a  b  |
                    # Find Jacobian           |                     |=|        |
        #             |   dzerror/dlat    dzerror/dalt  | |  c  d  |
        #             --                               -- --      -- 

        aa = drdl*clat - (r_n + tempalt)*slat
        bb = clat
        cc = (1 - NAV_E2)*(drdl*slat + r_n*clat)
        dd = slat

        #  Apply correction = inv(Jacobian)*errorvector

        invdet = 1.0/(aa*dd - bb*cc)
        templat = templat - invdet*(+dd*rhoerror -bb*zerror)
        tempalt = tempalt - invdet*(-cc*rhoerror +aa*zerror)

    wlat = templat*rad2deg;
    walt = tempalt;
    


    return wlat,wlon,walt

def main():
    global pose_pub
    rospy.init_node('sep_to_utm')
    
    pose_pub = rospy.Publisher("/septentrio/utm_pose", PoseStamped,queue_size=100)
    
    rospy.Subscriber("/septentrio/pvt_cartesian", PvtCartesianMsg, pos_callback)
    rospy.Subscriber("/septentrio/attitude_euler", AttitudeEulerMsg, att_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass