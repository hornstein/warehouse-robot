#!/usr/bin/env python

import sys
import os
import csv

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

import ctypes
import struct

import numpy as np
import cv2

import math

import time


def callback(point_cloud):

    article_nr = raw_input("Enter article number: ")

    x = []
    y = []
    z = [] 
    r = []
    g = []
    b = []
    rgb = []   

    for p in pc2.read_points(point_cloud): 
        if (p[0]>-0.5 and p[0]<0.5 and p[1]>-0.35 and p[1]<0.35 and p[2]>0.5 and p[2]<1.3):
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
            rgb = p[3] 
            r.append( (rgb & 0x00FF0000)>> 16)
            g.append( (rgb & 0x0000FF00)>> 8 )
            b.append( (rgb & 0x000000FF) )
    
    # find item edges

    color_image = np.zeros((350,500,3), np.uint8)
    depth_image = np.zeros((350,500,3), np.uint8)
    
    for n in range(1,len(z)):
        color_image[int((y[n]+0.35)*350),int((x[n]+0.5)*500)] = (b[n],g[n],r[n])
        depth_image[int((y[n]+0.35)*350),int((x[n]+0.5)*500)] = ((1.3-z[n])*100, (1.3-z[n])*100, (1.3-z[n])*100)

    #cv2.imwrite('color_image.png', color_image)
    #gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
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


    #depth_image[min_y, min_x]=(255, 0, 0)
    #depth_image[min_y, max_x]=(255, 0, 0)
    #depth_image[max_y, min_x]=(255, 0, 0)
    #depth_image[max_y, max_x]=(255, 0, 0)
    #cv2.imwrite('depth_image.png', depth_image)


    # Create new image only containing item

    item_image = np.zeros((max_y-min_y, max_x-min_x,3), np.uint8)
    item_image = color_image[min_y:max_y, min_x:max_x]
    

    # Transform image values back to camera coordinates

    min_x = min_x/500.0-0.5
    max_x = max_x/500.0-0.5
    min_y = min_y/350.0-0.35
    max_y = max_y/350.0-0.35
    box_z = 1.3-box_z/100.0



    box = []
    box.append([min_y+1.1, min_x, -box_z+1.3])
    box.append([min_y+1.1, max_x, -box_z+1.3])
    box.append([max_y+1.1, max_x, -box_z+1.3])
    box.append([max_y+1.1, min_x, -box_z+1.3])

    print "Item coordinates:"
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

    raw_input("Verify box and press and  Enter to continue...")

    # Store data


    f = open("data/calibration/table_distance.txt", "r")

    table_height = float(f.read())

    directory = 'data/' + article_nr

    if not os.path.exists(directory):
        os.makedirs(directory)

    filename = directory + '/dimensions.csv'

    with open(filename, 'w') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        csv_writer.writerow([str(max_x-min_x), str(max_y-min_y), str(table_height-box_z)])

    filename = directory + '/image.png'
    cv2.imwrite(filename, item_image)



    filename = directory + '/pointcloud.csv'

    with open(filename, 'w') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for n in range(1,len(z)):
            csv_writer.writerow([x[n], y[n], z[n], r[n], g[n], b[n]])

 
    raw_input("Press Enter to continue...")


def listener():
    print "start listener"
    rospy.init_node('measure_article', anonymous=False)
    print "Subscribe"
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    print "Subscribed"
    rospy.spin()

if __name__ == '__main__':
    listener()
