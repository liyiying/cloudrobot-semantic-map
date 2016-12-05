# cloudrobot-semantic-map
Robot can build a semantic map on object level based on a hybrid cloud(mission cloud & public cloud). The mission cloud can deal with the work specific to the robot or the robot is familiar with. If the task is beyond the robot's capability, the mission cloud will seak help from the public cloud, because the public cloud has more knowledge from the Internet and so on. For example, robot can build a semantic map of a room via recognizing the objects to realize the scene understanding. The mission cloud can recognize the objects it has been trained in advance, however, when the object is beyond its knowledge, it may transfer the object image to the public cloud, like [CloudSight](http://www.cloudsightapi.com/). This work is an implementation of robot semantic map buiding based on a hybrid cloud architecture. The object recognition engine on the mission cloud is based on [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn) and the public cloud uses the Internet open object recognition service [CloudSight](http://www.cloudsightapi.com/). This work is a fusion of the object recognition and geometry map to build the semantic map of a room. BTW, the only hybrid cloud object recognition work is also available(https://github.com/liyiying/py-faster-rcnn) and can be used in many aspects not only the semantic map.

## Requirements:cloud server
The requirements is the same as [Faster R-CNN requirements](https://github.com/rbgirshick/py-faster-rcnn/blob/master/README.md). (Because the clous server should run Faster R-CNN on it)

## Requirements:robot
We work this based on [Turtlebot robot](http://wiki.ros.org/Robots/TurtleBot).

## How to use:
* git clone https://github.com/liyiying/cloudrobot-semantic-map.git. The codes in 'semantic_map_robot' are deployed and run on robot, and the codes in 'semantic_map_cloud' are on cloud server.
* Install ROS on both Turtlebot and cloud server. See [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) for details.
* Install software onto the turtlebot.See [Turtlebot installation](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) for details.
* Get turtlebot and cloud server chat to each other. Let them under one Master's control. Network Configuration of Turtlebot and pc, see [Network Configuration](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration) for details.

### On TurtleBot:
* Put turtlebot_follower package to your own ROS workspace(~/catkin_ws/src).Then build the package. See [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages#ROS.2BAC8-Tutorials.2BAC8-rosbuild.2BAC8-BuildingPackages.Building_Packages) for details.

* Modify gmapping_demo.launch
```
sudo gedit /opt/ros/indigo/share/turtlebot_navigation/launch/gmapping_demo.launch
```
 remove codes below 
```
<include file="$(find turtlebot_bringup)/launch/3dsensor.launch">
    <arg name="rgb_processing" value="false" />
    <arg name="depth_registration" value="false" />
    <arg name="depth_processing" value="false" />
    
    <!-- We must specify an absolute topic name because if not it will be prefixed by "$(arg camera)".
         Probably is a bug in the nodelet manager: https://github.com/ros/nodelet_core/issues/7 --> 
    <arg name="scan_topic" value="/scan" />
  </include>
```

* Set ROS workspace in ~/.bashrc
```
sudo gedit ~/.bashrc
```
 add this
```
source ~/catkin_ws/devel/setup.bash
```

* Bringup Turtlebot
```python
roslaunch turtlebot_bringup minimal.launch
```

* Calculate Turtlebot's pose
```python
cd $semantic_map_robot
python quat_to_angle_xy.py
```

* Calculate the distance from Turtlebot to object points
```
roslaunch turtlebot_follower follower.launch
```

* Build SLAM map
```
roslaunch turtlebot_navigation gmapping_demo.launch
```
 the map can be seen on Turtlebot or cloud server
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

### On Cloud Server:
* Put cs.py and demo_cs.py to your $Faster R-CNN/tools, and install [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn).

* 
```
cd $semantic_map_cloud
rosrun image_view image_saver image:=/camera/rgb/image_raw _save_all_image:=false _filename_format:=foo.jpg __name:=image_saver
```

* Calculate the object's position on the map
```
cd $semantic_map_cloud
python object_position4.py
```

* Control Turtlebot by keyboard to build semantic map
```
ssh $TurtlebotName@ <Turtlebot_IP>
roslaunch turtlebot_teleop keyboard_teleop.launch
```






