# Picking controller



## Create Python package for controller 

```
cd ws_abb/src 

catkin_create_pkg controller sensor_msgs rospy roscpp 

cd .. 

catkin build 

cd src/controller/scripts/ 

chmod +x listener.py 
```

## Startup sequence: 

```
cd ws_abb/ 

source devel/setup.bash 
```
 

Terminal 1: 
```
roscore 
```
Terminal 2: 
```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud 
```
Terminal 3: 
```
roslaunch abb_irb2400_support test.launch 
```
Terminal 4: 
```
rosrun tf static_transform_publisher 1.1 0 1.3 1 0 -1 0 /base_link /camera_link 100 
```
Terminal 5: 
```
rosrun controller controller_test.py 
```
Terminal 6: 
```
rosrun controller robot_server.py 
```


## Coordinates for calibration: 

Home = [400, 0, 1000] 

Point 1 = [860, 230, 560] 

Point 2 = [860, -215, 560] 