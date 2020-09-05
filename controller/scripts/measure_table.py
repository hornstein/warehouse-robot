#!/usr/bin/env python

import sys
import os

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

#from visualization_msgs.msg import Marker

#from geometry_msgs.msg import Point
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

#import ctypes
#import struct

import numpy as np
#import cv2

#import math

#import time


def callback(point_cloud):

    print "received point cloud message"

    x = []
    y = []
    z = [] 
    r = []
    g = []
    b = []
    rgb = []   
    
    for p in pc2.read_points(point_cloud): 
        if (p[0]>-0.1 and p[0]<0.1 and p[1]>-0.1 and p[1]<0.1 and p[2]>0):
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
            rgb = p[3] 
            r.append( (rgb & 0x00FF0000)>> 16)
            g.append( (rgb & 0x0000FF00)>> 8 )
            b.append( (rgb & 0x000000FF) )
    
    # find distance to table
    table_z = np.median(z)
    print table_z

    # store table distance
    directory = 'data/calibration'

    if not os.path.exists(directory):
        os.makedirs(directory)

    f = open("data/calibration/table_distance.txt","w+")
    f.write(str(table_z))
    f.close()

    raw_input("Press Enter to continue...")


def listener():
    print "start listener"
    rospy.init_node('measure_table', anonymous=False)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    print "Subscribed to point cloud"
    rospy.spin()

if __name__ == '__main__':
    listener()
