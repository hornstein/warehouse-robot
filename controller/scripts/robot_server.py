#!/usr/bin/env python

import sys
import rospy
import abb

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from controller.srv import *


def callback(req):

    print "received message"
    #rospy.loginfo(data)
    x=req.cartesian_point.x*1000
    y=req.cartesian_point.y*1000
    z=req.cartesian_point.z*1000

    print x, y, z

    if(x<1400 and x>600 and y<200 and y>-200 and z>700 and z<1101):
        R.set_cartesian([[x,y,z], [0,0,1,0]])
    
    joint_pos=R.get_joints()
    print joint_pos
    joint_values=[float(x)*0.01745 for x in joint_pos]
    print joint_values

    pub=rospy.Publisher('joint_states', JointState, queue_size='10')
    msg=JointState()
    msg.header=Header()
    msg.header.stamp=rospy.Time.now()
    msg.name=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    msg.position=joint_values
    msg.velocity=[]
    msg.effort=[]
    pub.publish(msg)

    #print "Received point [%s, %s, %s]"%(req.cartesian_point.x, req.cartesian_point.y, req.cartesian_point.z)
    return MoveRobotResponse(1)

def robot_server():
    rospy.init_node('robot_server')
    s = rospy.Service('move_robot', MoveRobot, callback)
    print "Ready to receive point."
    rospy.spin()

if __name__ == "__main__":
    R=abb.Robot(ip='192.168.125.1')
    robot_server()
