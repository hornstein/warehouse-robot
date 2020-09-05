#! /usr/bin/env python

import os
import rospy
import threading

from flask import Flask
from flask import request, Response, jsonify
from flask import render_template, send_from_directory


from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy as np
import csv
import cv2
import math
import time

from threading import Lock, Thread

import pickle

import abb

import requests



lock = Lock()
pc = PointCloud2()
bounding_box = []
article_dim = []
article_image_length = 0
article_image_width = 0
article_image_height = 0
box_dim = []
picking_bounding_boxes = []
picking_points = []
global R
global depth_image
global content_image
content_image_offset_x = 0
content_image_offset_y = 0


app = Flask(__name__, static_url_path='')


@app.route('/')
def start():
    return render_template('index.html', name='Bluebottle robot interface')


@app.route('/archive/<path:filename>')
def download_file(filename):
    print "Sending file"
    if filename.endswith('.png'):
        return send_from_directory('archive', filename, mimetype = 'image/png')
    elif filename.endswith('.jpg'):
        return send_from_directory('archive', filename, mimetype = 'image/jpg') 
    elif filename.endswith('.js'):
        print "sending " + filename
        return send_from_directory('archive', filename) 
    return send_from_directory('archive', filename)


@app.route('/calibrate')
def calibrate():
    find_table()
    return 'Table calibrated!'


@app.route('/init_robot')
def init_robot():
    global R
    print 'initializing robot'
    R=abb.Robot(ip='192.168.125.1')
    return 'Robot initiated!'

@app.route('/home_robot')
def home_robot():
    global R
    R.set_speed([300,150,150,150])
    print 'home robot'
    x = 600
    y = 0
    z = 1100
    R.set_cartesian([[x,y,z], [0,0,1,0]])
    joint_pos=R.get_joints()
    print joint_pos
    joint_values=[float(x)*0.01745 for x in joint_pos]
    print joint_values
    list = []
    list.append({'link_1': joint_values[0], 'link_2': joint_values[1], 'link_3': joint_values[2], 'link_4': joint_values[3], 'link_5': joint_values[4], 'link_6': joint_values[5]})
    return jsonify(results = list)


@app.route('/picking_all')
def picking_all():
    article_id = request.args.get('id')
    print "id = " + article_id
    print 'picking_all'

    global picking_points
    global R
    R.set_speed([300,150,150,150])

    f = open("data/calibration/table_distance.txt", "r")
    table_height = float(f.read())
    print table_height
    
    measure_box()
    find_picking_points(article_id)

    print "start picking"
    print picking_points[0][2]
    print 1.3-table_height

    while picking_points[0][2]>1.3-table_height+0.02:

        print 'pick up'
        x = picking_points[0][0]*1000
        y = picking_points[0][1]*1000
        z = 1100
        R.set_cartesian([[x,y,z], [0,0,1,0]])
        
        print 'pick down'
        z = picking_points[0][2]*1000+110
        R.set_cartesian([[x,y,z], [0,0,1,0]])
        time.sleep(1)
        r = requests.get("http://192.168.33.1/relay/0?turn=on")
        time.sleep(2)
        
        print 'pick up'
        z = 1100
        R.set_cartesian([[x,y,z], [0,0,1,0]])

        print 'pack position'
        R.set_cartesian([[600,-1000,1000], [0,0,1,0]])
        R.set_cartesian([[600,-1000,1000], [0.0, 0.0, -1.0, -0.40]])
        time.sleep(1)
        r = requests.get("http://192.168.33.1/relay/0?turn=off")
        time.sleep(2)

        print 'home robot'
        x = 600
        y = 0
        z = 1100
        R.set_cartesian([[x,y,z], [0,0,1,0]])

        measure_box()
        find_picking_points(article_id)
    

    joint_pos=R.get_joints()
    print joint_pos
    joint_values=[float(x)*0.01745 for x in joint_pos]
    print joint_values
    list = []
    list.append({'link_1': joint_values[0], 'link_2': joint_values[1], 'link_3': joint_values[2], 'link_4': joint_values[3], 'link_5': joint_values[4], 'link_6': joint_values[5]})
    return jsonify(results = list)


@app.route('/picking_up')
def picking_up():
    global R
    print 'picking_up'
    x = picking_points[0][0]*1000
    y = picking_points[0][1]*1000
    z = 1100
    R.set_cartesian([[x,y,z], [0,0,1,0]])
    
    z = picking_points[0][2]*1000+110
    R.set_cartesian([[x,y,z], [0,0,1,0]])
    time.sleep(1)
    r = requests.get("http://192.168.33.1/relay/0?turn=on")
    time.sleep(2)

    z = 1100
    R.set_cartesian([[x,y,z], [0,0,1,0]])

    R.set_cartesian([[600,-1000,1000], [0,0,1,0]])
    R.set_cartesian([[600,-1000,1000], [0.0, 0.0, -1.0, -0.40]])
    time.sleep(1)
    r = requests.get("http://192.168.33.1/relay/0?turn=off")
    time.sleep(2)

    print 'home robot'
    x = 600
    y = 0
    z = 1100
    R.set_cartesian([[x,y,z], [0,0,1,0]])

    joint_pos=R.get_joints()
    
    print joint_pos
    joint_values=[float(x)*0.01745 for x in joint_pos]
    print joint_values
    list = []
    list.append({'link_1': joint_values[0], 'link_2': joint_values[1], 'link_3': joint_values[2], 'link_4': joint_values[3], 'link_5': joint_values[4], 'link_6': joint_values[5]})
    return jsonify(results = list)


# @app.route('/picking_down')
# def picking_down():
#     global R
#     print 'picking_down'
#     x = picking_points[0][0]*1000
#     y = picking_points[0][1]*1000
#     z = picking_points[0][2]*1000+150
#     R.set_cartesian([[x,y,z], [0,0,1,0]])
#     joint_pos=R.get_joints()
#     print joint_pos
#     joint_values=[float(x)*0.01745 for x in joint_pos]
#     print joint_values
#     list = []
#     list.append({'link_1': joint_values[0], 'link_2': joint_values[1], 'link_3': joint_values[2], 'link_4': joint_values[3], 'link_5': joint_values[4], 'link_6': joint_values[5]})
#     return jsonify(results = list)



@app.route('/move_robot')
def test_robot():
    x = int(request.args.get('x'))
    y = int(request.args.get('y'))
    z = int(request.args.get('z'))
    global R
    #R.set_cartesian([[600,0,1100], [0,0,1,0]])
    #R.set_cartesian([[650,0,1100], [0,0,1,0]])
    #R.set_cartesian([[700,0,1100], [0,0,1,0]])
    #R.set_cartesian([[750,0,1100], [0,0,1,0]])
    #R.set_cartesian([[800,0,1100], [0,0,1,0]])
    #R.set_cartesian([[850,0,1100], [0,0,1,0]])
    #R.set_speed([30,15,15,15])
    #R.set_cartesian([[900,0,1100], [0,0,1,0]])
    #R.set_cartesian([[950,0,1100], [0,0,1,0]])
    #R.set_speed([30,15,15,15])
    print 'test robot'
    R.set_cartesian([[x,y,z], [0,0,1,0]])
    joint_pos=R.get_joints()
    print joint_pos
    joint_values=[float(x)*0.01745 for x in joint_pos]
    print joint_values
    list = []
    list.append({'link_1': joint_values[0], 'link_2': joint_values[1], 'link_3': joint_values[2], 'link_4': joint_values[3], 'link_5': joint_values[4], 'link_6': joint_values[5]})
    return jsonify(results = list)



@app.route('/article')
def article():
    filename = request.args.get('filename')
    filename = filename + '.bin'
    print filename
    measure_box()
    global bounding_box
    global article_dim
    article_dim = bounding_box
    with open(filename, 'wb') as f:
        pickle.dump(article_dim, f)
    list = []
    nr_of_points=0
    for n in range(0,len(article_dim)):
        list.append({'x': article_dim[n][0], 'y': article_dim[n][1], 'z': article_dim[n][2]})
        nr_of_points=nr_of_points+1
    
    print "Number of article points sent: "
    print nr_of_points
    return jsonify(results = list)
    #return 'Measured article!'

@app.route('/box')
def box():
    measure_box()
    global bounding_box
    global box_dim
    box_dim = bounding_box
    list = []
    nr_of_points=0
    for n in range(0,len(box_dim)):
        list.append({'x': box_dim[n][0], 'y': box_dim[n][1], 'z': box_dim[n][2]})
        nr_of_points=nr_of_points+1
    
    print "Number of box points sent: "
    print nr_of_points
    return jsonify(results = list)


@app.route('/picking_points')
def get_picking_points():
    article_id = request.args.get('id')
    print "id = " + article_id
    find_picking_points(article_id)
    global picking_points
    list = []
    nr_of_points=0
    for n in range(0,len(picking_points)):
        list.append({'x': picking_points[n][0], 'y': picking_points[n][1], 'z': picking_points[n][2]})
        nr_of_points=nr_of_points+1
    
    print "Number of picking points sent: "
    print nr_of_points
    return jsonify(results = list)


@app.route('/save_pointcloud')
def save_point_cloud():
    filename = request.args.get('filename')
    filename = filename + '.bin'
    print filename
    lock.acquire()   
    with open(filename, 'wb') as f:
        pickle.dump(pc, f)
    lock.release() 
    return 'Saved pointcloud!'


@app.route('/load_pointcloud')
def load_point_cloud():
    filename = request.args.get('filename')
    filename = filename + '.bin'
    print filename
    global pc
    lock.acquire()   
    with open(filename, 'rb') as f:
        pc = pickle.load(f)
    lock.release() 
    return 'Loaded pointcloud!'


@app.route('/pointcloud')
def get_point_cloud():

    x = []
    y = []
    z = [] 
    r = []
    g = []
    b = []
    rgb = [] 
    lock.acquire()   
    for p in pc2.read_points(pc): 
        if (p[0]>-0.5 and p[0]<0.5 and p[1]>-0.35 and p[1]<0.35 and p[2]>0.1 and p[2]<1.3):
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
            rgb = p[3] 
            r.append( (rgb & 0x00FF0000)>> 16)
            g.append( (rgb & 0x0000FF00)>> 8 )
            b.append( (rgb & 0x000000FF) )

    lock.release() 

    print "number of points in pointcloud: "
    print len(z)

    list = []
    nr_of_points=0
    for n in range(0,len(z), len(z)/10000):
        # list.append({'x': x[n], 'y': y[n], 'z': z[n], 'r': r[n], 'g': g[n], 'b': b[n]})
        list.append({'x': y[n]+1.1, 'y': x[n], 'z': -z[n]+1.3, 'r': r[n], 'g': g[n], 'b': b[n]})
        nr_of_points=nr_of_points+1
    
    print "Number of points sent: "
    print nr_of_points
    return jsonify(results = list)





def callback(point_cloud):
    lock.acquire() 
    global pc
    pc = point_cloud
    lock.release() 



def find_table():

    x = []
    y = []
    z = [] 
    r = []
    g = []
    b = []
    rgb = [] 
    lock.acquire()   
    #print pc
    for p in pc2.read_points(pc): 
        if (p[0]>-0.5 and p[0]<0.5 and p[1]>-0.35 and p[1]<0.35 and p[2]>0):
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
            rgb = p[3] 
            r.append( (rgb & 0x00FF0000)>> 16)
            g.append( (rgb & 0x0000FF00)>> 8 )
            b.append( (rgb & 0x000000FF) )
    lock.release() 

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


def measure_box():

    global bounding_box
    global depth_image
    global content_image
    global content_image_offset_x
    global content_image_offset_y

    print "measuring box"

    f = open("data/calibration/table_distance.txt", "r")
    table_height = float(f.read())

    print "reading point cloud"

    x = []
    y = []
    z = [] 
    r = []
    g = []
    b = []
    rgb = []   

    lock.acquire() 
    for p in pc2.read_points(pc): 
        if (p[0]>-0.5 and p[0]<0.5 and p[1]>-0.35 and p[1]<0.35 and p[2]>0.1 and p[2]<table_height):
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
            rgb = p[3] 
            r.append( (rgb & 0x00FF0000)>> 16)
            g.append( (rgb & 0x0000FF00)>> 8 )
            b.append( (rgb & 0x000000FF) )
    lock.release() 


    # find item edges

    print "find item edges"

    color_image = np.zeros((350,500,3), np.uint8)
    depth_image = np.zeros((350,500,3), np.uint8)
    
    for n in range(1,len(z)):
        color_image[int((y[n]+0.35)*500),int((x[n]+0.5)*500)] = (b[n],g[n],r[n])
        depth_image[int((y[n]+0.35)*500),int((x[n]+0.5)*500)] = ((table_height-z[n])*500, (table_height-z[n])*500, (table_height-z[n])*500)

    cv2.imwrite('depthimage.png', depth_image)
    cv2.imwrite('colorimage.png', color_image)
    
    min_x = 500
    max_x = 0
    min_y = 350
    max_y = 0
    min_height = 5
    min_ratio = 0.6

    # Shrink bounding box
    min_x = 150
    max_x = 375
    # min_y = 50
    # max_y = 325
    min_y = 0
    max_y = 349
    print min_x, min_y, max_x, max_y

    print "shrinking bounding box"

    ratio=0
    count=0.0
    while count<20 and min_x<max_x:
        count=0.0
        edge_z = []
        for y3 in range(min_y, max_y):
            pixel_b, pixel_g, pixel_r = depth_image[y3, min_x]
            if(pixel_b>min_height):
                count = count + 1.0
                edge_z.append(pixel_b)
        min_x = min_x + 1

    box_z = np.median(edge_z)
    print min_x

    ratio=0
    count=0.0
    while count<20 and max_x>min_x:
        count=0.0
        edge_z = []
        for y3 in range(min_y, max_y):
            pixel_b, pixel_g, pixel_r = depth_image[y3, max_x]
            if(pixel_b>min_height):
                count = count + 1.0
                edge_z.append(pixel_b)
        max_x = max_x - 1
    
    box_z = max(box_z, np.median(edge_z))
    print max_x

    ratio=0
    count=0.0
    while count<20 and min_y<max_y:
        count=0.0
        edge_z = []
        for x3 in range(min_x, max_x):
            pixel_b, pixel_g, pixel_r = depth_image[min_y, x3]
            if(pixel_b>min_height):
                count = count + 1.0
                edge_z.append(pixel_b)
        min_y = min_y + 1

    box_z = max(box_z, np.median(edge_z))
    print min_y


    ratio=0
    count=0.0
    while count<20 and max_y>min_y:
        count=0.0
        edge_z = []
        for x3 in range(min_x, max_x):
            pixel_b, pixel_g, pixel_r = depth_image[max_y, x3]
            if(pixel_b>min_height):
                count = count + 1.0
                edge_z.append(pixel_b)
        max_y = max_y - 1

    print max_y

    box_z = max(box_z, np.median(edge_z))
    print "old box_z:"
    print box_z
    # Create new image only containing item

    content_image = depth_image[min_y:max_y, min_x:max_x]
    content_image_offset_x = min_x
    content_image_offset_y = min_y

    box_z=content_image.mean()   
    print "new box_z:"
    print box_z

    # Transform image values back to camera coordinates

    min_x = min_x/500.0-0.5
    max_x = max_x/500.0-0.5
    min_y = min_y/500.0-0.35
    max_y = max_y/500.0-0.35
    box_z = table_height-box_z/500.0


    # Transform camera coordinates to robot coordinates

    box = []
    box.append([min_y+1.1, min_x, -box_z+1.3])
    box.append([min_y+1.1, max_x, -box_z+1.3])
    box.append([max_y+1.1, max_x, -box_z+1.3])
    box.append([max_y+1.1, min_x, -box_z+1.3])

    bounding_box = box

    print box

    #publish_marker()
    #store_data()


# def store_data():

#     # TO DO: change this to robot coordinates using bounding_box
#     # TO DO: store as blob instead of csv

#     global bounding_box

#     f = open("data/calibration/table_distance.txt", "r")

#     table_height = float(f.read())

#     directory = 'data/' + article_nr

#     if not os.path.exists(directory):
#         os.makedirs(directory)

#     filename = directory + '/dimensions.csv'

#     with open(filename, 'w') as csvfile:
#         csv_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
#         csv_writer.writerow([str(max_x-min_x), str(max_y-min_y), str(table_height-box_z)])

#     filename = directory + '/image.png'
#     cv2.imwrite(filename, item_image)

#     filename = directory + '/pointcloud.csv'

#     with open(filename, 'w') as csvfile:
#         csv_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
#         for n in range(1,len(z)):
#             csv_writer.writerow([x[n], y[n], z[n], r[n], g[n], b[n]])


def publish_marker_rviz():

    global bounding_box

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
    first_line_point.x = bounding_box[0][0]
    first_line_point.y = bounding_box[0][1]
    first_line_point.z = bounding_box[0][2]
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = bounding_box[1][0]
    second_line_point.y = bounding_box[1][1]
    second_line_point.z = bounding_box[1][2]
    marker.points.append(second_line_point)
    # third point
    third_line_point = Point()
    third_line_point.x = bounding_box[2][0]
    third_line_point.y = bounding_box[2][1]
    third_line_point.z = bounding_box[2][2]
    marker.points.append(third_line_point)
    # forth point
    forth_line_point = Point()
    forth_line_point.x = bounding_box[3][0]
    forth_line_point.y = bounding_box[3][1]
    forth_line_point.z = bounding_box[3][2]
    marker.points.append(forth_line_point)
    marker.points.append(first_line_point)
    rospy.sleep(3.0) 
    marker_publisher.publish(marker)


def find_picking_points(article_id):

    global box_dim
    global article_dim
    global content_image
    global content_image_offset_x
    global content_image_offset_y
    global picking_points

    filename = article_id + '.bin'
    print filename
    with open(filename, 'rb') as f:
        article_dim = pickle.load(f)

    print "article dim:"
    print article_dim

    f = open("data/calibration/table_distance.txt", "r")
    table_height = float(f.read())

    print "Finding picking points..."

    print "box_dim"
    print box_dim

    print "article_dim"
    print article_dim

    box_width = (box_dim[2][0]-box_dim[0][0])*500
    print box_width
    box_length = (box_dim[2][1]-box_dim[0][1])*500
    print box_length
    box_height = (box_dim[0][2]-(1.3-table_height))*500
    print box_height

    article_width = (article_dim[2][0]-article_dim[0][0])*500
    print article_width
    article_length = (article_dim[2][1]-article_dim[0][1])*500
    print article_length
    article_height = (article_dim[0][2]-(1.3-table_height))*500
    print article_height

    article_width = min(article_width, box_width)
    article_length = min(article_length, box_length)

    print "Adjusted article width and length:"
    print article_width
    print article_length

    print "Nr of articles per dim:"
    print (box_width/article_width)
    print round(box_width/article_width)
    print (box_length/article_length)
    print round(box_length/article_length)
    print (box_height/article_height)
    print round(box_height/article_height)

    # Create image of box content

    #lineThickness = 2
    #cv2.line(content_image, (int(article_length/2), int(box_width-article_width/2)), (100, 100), (0,255,0), lineThickness)

    cv2.imwrite('contentimage.png', content_image)


    height, width, channels = content_image.shape

    print height
    print width

    gray_image = cv2.cvtColor(content_image, cv2.COLOR_BGR2GRAY)

    # Find largest Z-value inside box
    all_z = []
    print "Find highest z-value inside box"
    max_z = 0
    for y in range(20,height-20):
        for x in range(20,width-20):
            z=gray_image[y,x]
            max_z = max(max_z, z)
            all_z.append(z)

    print "highest point"
    print max_z
    all_z.sort(reverse=True)
    max_z = all_z[int(article_width*article_length/4)]
    print max_z
    print "article height"
    print article_height

    #article_height=max(article_height, 20)
    offset = max_z-int(0.8*article_height)
    #offset = max(offset, 10)

    # (thresh, binary_image) = cv2.threshold(gray_image, box_height-article_height/2, 255, cv2.THRESH_BINARY)
    (thresh, binary_image) = cv2.threshold(gray_image, offset, 255, cv2.THRESH_BINARY)

    cv2.imwrite('binaryimage.png', binary_image)

    # Search binary image

    print "Starting search..."

    x = int(article_length/2)+1+int((box_length-round(box_length/article_length)*article_length)/2)
    #y = int(box_width-article_width/2)-1
    y = int(box_width-article_width/2)

    print "y"
    print y

    print "article_width/2"
    print article_width/2

    threshold=0.8
    found = False
    
    while not found and x<box_length:
        print "Searching for x"
        while not found and y>=int(article_width/2):
            roi = binary_image[y-int(article_width/2):y+int(article_width/2), x-int(article_length/2):x+int(article_length/2)]
            if sum(sum(roi/255.0))/(article_width*article_length) > threshold:
                found = True
            y=y-1
            print y
        if not found:
            print "Not found for x"
            y = int(box_width-article_width/2)-1
            x = x + int(article_length)


    # Do a global search if not yet found
    if not found:
        print "Doing global search..."
        y = int(box_width-article_width/2)-1
        x = int(article_length/2)+1

    while not found and x<box_length-article_length/2:
        while not found and y>article_width/2:
            roi = binary_image[y-int(article_width/2):y+int(article_width/2), x-int(article_length/2):x+int(article_length/2)]
            if sum(sum(roi/255.0))/(article_width*article_length) > threshold:
                print "Found picking point"
                found = True
            y=y-1
        if not found:
            y = int(box_width-article_width/2)-1
            x = x + 1


    print "Finished search"
    print x
    print y

    # Get Z from depth-image

    z = content_image[y, x]
    z = z[0]
    print z

    # Transform image values back to camera coordinates


    cam_x = (x+content_image_offset_x)/500.0-0.5
    cam_y = (y+content_image_offset_y)/500.0-0.35
    cam_z = table_height-z/500.0


    # Transform camera coordinates to robot coordinates

    picking_points = []
    picking_points.append([cam_y+1.1, cam_x, -cam_z+1.3])
    
    print picking_points



if __name__ == '__main__':
    #threading.Thread(target=lambda: rospy.init_node('measure_table', anonymous=False)).start()
    rospy.init_node('measure_table', anonymous=False)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback)
    app.run()
    #app.run(host='0.0.0.0', port=8081)
    rospy.spin()

