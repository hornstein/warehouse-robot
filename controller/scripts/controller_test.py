#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

from visualization_msgs.msg import Marker


from geometry_msgs.msg import Point
from controller.srv import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import ctypes
import struct

import numpy as np
import cv2

import math

import time


def move_robot_client(x, y, z):
    print "waiting for service"
    rospy.wait_for_service('move_robot')
    print "found service!"
    try:
        move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
        msg=Point()
        msg.x=x
        msg.y=y
        msg.z=z
        resp1 = move_robot(msg)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def callback(point_cloud):

    print "received message"

    x = []
    y = []
    z = [] 
    r = []
    g = []
    b = []
    rgb = []   
    for p in pc2.read_points(point_cloud, skip_nans=True): 
        if (p[0]>-0.5 and p[0]<0.5 and p[1]>-0.35 and p[1]<0.35 and p[2]>0.5 and p[2]<1.3):
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
            test = p[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            b.append( (pack & 0x00FF0000)>> 16 )
            g.append( (pack & 0x0000FF00)>> 8 )
            r.append( (pack & 0x000000FF) )
    
    # find box edges

    color_image = np.zeros((350,500,3), np.uint8)
    depth_image = np.zeros((350,500,3), np.uint8)
    #count=0
    for n in range(1,len(z)):
        color_image[int((y[n]+0.35)*350),int((x[n]+0.5)*500)] = (b[n],g[n],r[n])
        depth_image[int((y[n]+0.35)*350),int((x[n]+0.5)*500)] = ((1.3-z[n])*100, (1.3-z[n])*100, (1.3-z[n])*100)
    cv2.imwrite('color_image.png', color_image)
    gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
    #median = cv2.medianBlur(gray,3)
    #cv2.imwrite('depth_image.png', median)

    min_x = 500
    max_x = 0
    min_y = 350
    max_y = 0
    box_z = 0
    for xx in range(1, 400):
        for yy in range(1, 350):
            pixel_b, pixel_g, pixel_r = depth_image[yy, xx]
            if(pixel_b>60 and pixel_b<70):
                min_x = min(min_x, xx)
                max_x = max(max_x, xx)
                min_y = min(min_y, yy)
                max_y = max(max_y, yy)
                box_z = pixel_b


    # Shrink bounding box
    ratio=0
    while ratio<0.8:
        count=0.0
	for y3 in range(min_y, max_y):
            pixel_b, pixel_g, pixel_r = depth_image[y3, min_x]
            if(pixel_b>60 and pixel_b<70):
                count = count + 1.0
        print count
        ratio=count/(max_y-min_y)
        print ratio
        min_x = min_x + 1

    ratio=0
    while ratio<0.8:
        count=0.0
	for y3 in range(min_y, max_y):
            pixel_b, pixel_g, pixel_r = depth_image[y3, max_x]
            if(pixel_b>60 and pixel_b<70):
                count = count + 1.0
        print count
        ratio=count/(max_y-min_y)
        print ratio
        max_x = max_x - 1

    ratio=0
    while ratio<0.8:
        count=0.0
	for x3 in range(min_x, max_x):
            pixel_b, pixel_g, pixel_r = depth_image[min_y, x3]
            if(pixel_b>60 and pixel_b<70):
                count = count + 1.0
        print count
        ratio=count/(max_x-min_x)
        print ratio
        min_y = min_y + 1

    ratio=0
    while ratio<0.8:
        count=0.0
	for x3 in range(min_x, max_x):
            pixel_b, pixel_g, pixel_r = depth_image[max_y, x3]
            if(pixel_b>60 and pixel_b<70):
                count = count + 1.0
        print count
        ratio=count/(max_x-min_x)
        print ratio
        max_y = max_y - 1


    depth_image[min_y, min_x]=(255, 0, 0)
    depth_image[min_y, max_x]=(255, 0, 0)
    depth_image[max_y, min_x]=(255, 0, 0)
    depth_image[max_y, max_x]=(255, 0, 0)
    cv2.imwrite('depth_image.png', depth_image)


    # Transform image values back to camera coordinates

    min_x = min_x/500.0-0.5
    max_x = max_x/500.0-0.5
    min_y = min_y/350.0-0.35
    max_y = max_y/350.0-0.35
    box_z = 1.3-box_z/100.0



    #min_x = 0
    #max_x = 0
    #min_y = 0
    #max_y = 0
    #box_z = 0

    #for n in range(1,len(z)):
    #    if(z[n]<0.8 and z[n]>0.5):
    #        min_x = min(min_x, x[n])
    #        max_x = max(max_x, x[n])
    #        min_y = min(min_y, y[n])
    #        max_y = max(max_y, y[n])
    #        box_z = z[n]

    box = []
    box.append([min_y+1.1, min_x, -box_z+1.3])
    box.append([min_y+1.1, max_x, -box_z+1.3])
    box.append([max_y+1.1, max_x, -box_z+1.3])
    box.append([max_y+1.1, min_x, -box_z+1.3])

    print "Box coordinates:"
    print min_x, min_y
    print max_x, max_y


    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size='10')
    
    marker = Marker()
    #marker.header=Header(frame_id='base_link')
    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = box[0][0]
    first_line_point.y = box[0][1]
    first_line_point.z = box[0][2]
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = box[1][0]
    second_line_point.y = box[1][1]
    second_line_point.z = box[1][2]
    marker.points.append(second_line_point)
    # third point
    third_line_point = Point()
    third_line_point.x = box[2][0]
    third_line_point.y = box[2][1]
    third_line_point.z = box[2][2]
    marker.points.append(third_line_point)
    # forth point
    forth_line_point = Point()
    forth_line_point.x = box[3][0]
    forth_line_point.y = box[3][1]
    forth_line_point.z = box[3][2]
    marker.points.append(forth_line_point)
    marker.points.append(first_line_point)
    rospy.sleep(3.0) 
    marker_publisher.publish(marker)

    print "Published marker"

    raw_input("Verify box and press Enter to continue...")

    # calculate picking positions

    #item_width = 0.09
    #item_height = 0.09
    #item_width = 0.27
    #item_height = 0.2
    item_width = 0.22
    item_height = 0.1

    px=[]
    py=[]
    pz=[]

    x_pos=min_x+item_width/2
    y_pos=min_y+item_height/2

    while(y_pos<max_y):
        while(x_pos<max_x):
            px.append(x_pos)
            py.append(y_pos)
            dist=9999
            closest_point=0
            for n in range(1,len(z)):
               if(math.sqrt((x[n]-x_pos)*(x[n]-x_pos)+(y[n]-y_pos)*(y[n]-y_pos))<dist):
                   dist=math.sqrt((x[n]-x_pos)*(x[n]-x_pos)+(y[n]-y_pos)*(y[n]-y_pos))
                   closest_point=n
            pz.append(z[closest_point])
            print x_pos, y_pos, z[closest_point]
            x_pos=x_pos+item_width
        x_pos=min_x+item_width/2
        y_pos=y_pos+item_height


    # transform to robot coordinates
    #transformation 1.1 0 1.3 1 0 -1 0
    rx=[]
    ry=[]
    rz=[]

    for n in range(0,len(pz)):
        rx.append(py[n]+1.1)
        ry.append(px[n]-0.0)
        rz.append(-pz[n]+1.3)

    print "Picking points in robot coordinates:"
    for n in range(0,len(rz)):
        print rx[n],ry[n],rz[n]

    # Publish a point
    pub=rospy.Publisher('picking_point', Point, queue_size=10)
    for n in range(0,len(rz)):

        marker = Marker()
        #marker.header=Header(frame_id='base_link')
        marker.header.frame_id = "/base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.pose.position.x = rx[n]
        marker.pose.position.y = ry[n]
        marker.pose.position.z = rz[n]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker_publisher.publish(marker)

        x=rx[n]
        y=ry[n]
        z=1.1
        print x, y, z
        raw_input("Verify picking position and press Enter to move robot...")
        move_robot_client(x, y, z)
        #time.sleep(5)
        x=rx[n]
        y=ry[n]
        z=rz[n]+0.14
        print x, y, z
        raw_input("Press Enter to move robot...")
        move_robot_client(x, y, z)
        #time.sleep(3)
        x=rx[n]
        y=ry[n]
        z=1.1
        print x, y, z
        raw_input("Press Enter to move robot...")
        move_robot_client(x, y, z)
        #time.sleep(3)
        x=0.7
        y=0
        z=1.1
        print x, y, z
        raw_input("Press Enter to move robot...")
        move_robot_client(x, y, z)
        #time.sleep(5)
        x=0.7
        y=0
        z=0.80
        print x, y, z
        raw_input("Press Enter to move robot...")
        move_robot_client(x, y, z)
        x=0.7
        y=0
        z=1.1
        print x, y, z
        raw_input("Press Enter to move robot...")
        move_robot_client(x, y, z)


    # create image from point cloud
    #blank_image = np.zeros((350,500,3), np.uint8)
    #count=0
    #for n in range(1,len(z)):
    #    count=count+1
    #    blank_image[int((y[n]+0.3)*350),int((x[n]+0.3)*500)] = (b[n],g[n],r[n])
    #print count
    #gray = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
    #median = cv2.medianBlur(gray,3)
    
    #cv2.imwrite('blank_image.png', blank_image)

    #do template matching
    #template = cv2.imread('template2.png', 0)

    #(h, w) = template.shape[:2]

    #result = cv2.matchTemplate(median, template, cv2.TM_SQDIFF_NORMED)

    #gpx = []
    #gpy = []
    #gpz = []
     
    #threshold = 0.018
    #loc = np.where( result < threshold)
    #print "number of found locations="
    #print len(zip(*loc[::-1]))
    #count=0
    #for pt in zip(*loc[::-1]):
    #    count=count+1
    #    print count
    #    for n in range(1,len(z),100):
    #        if (int((y[n]+0.2)*350)==int(pt[1] + h/2) and int((x[n]+0.5)*500)==int(pt[0] + w/2)):
    #            gpx.append(x[n])
    #            gpy.append(y[n])
    #            gpz.append(z[n])

    #print "starting plotting"
    #print len(gpx)

    #fig = plt.figure()
    #ax = plt.axes(projection='3d')
    #for n in range(1,len(z),100):
    #    bw=float(r[n]+g[n]+b[n])/765.0
    #    ax.scatter3D(x[n], y[n], z[n], c=[bw,bw,bw])
    #for n in range(1,len(gpx)):
    #    ax.scatter3D(gpx[n], gpy[n], gpz[n], c=[1,0,0])
        
    #plt.show()
    raw_input("Press Enter to continue...")


def listener():
    print "start listener"
    rospy.init_node('listener', anonymous=False)
    print "Subscribe"
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    print "Subscribed"
    rospy.spin()

if __name__ == '__main__':
    listener()
