#!/usr/bin/env python
import rospy
import math
import os
import sys
import save_image_once
import shutil
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from math import radians, copysign, sqrt, pow, pi

angle = 0.0
posex = 0.0
posey = 0.0
depthob1 = 0.0
depthob2 = 0.0
depthob3 = 0.0
depthob4 = 0.0
dist_ob1 = 0.0
dist_ob2 = 0.0
dist_ob3 = 0.0
dist_ob4 = 0.0
offx1 = 0.0
offy1 = 0.0
offx2 = 0.0
offy2 = 0.0
offx3 = 0.0
offy3 = 0.0
offx4 = 0.0
offy4 = 0.0
objectx1 = 0.0
objecty1 = 0.0
objectx2 = 0.0
objecty2 = 0.0
objectx3 = 0.0
objecty3 = 0.0
objectx4 = 0.0
objecty4 = 0.0

def callback1(data):
    #rospy.loginfo("turtlebot_angle : %f ", data.data)
    global angle
    angle = data.data
    #print(math.sin(angle))
    #print(angle)
def callback2(data):
    #rospy.loginfo("turtlebot_posex : %f ", data.data)
    global posex
    posex = data.data
def callback3(data):
    #rospy.loginfo("turtlebot_posey : %f ", data.data)
    global posey
    posey = data.data

def callback4(data):
    #rospy.loginfo("depthob1 : %f ", data.data) #right-down
    global depthob1
    depthob1 = data.data
    global dist_ob1
    dist_ob1 = 1.04*depthob1 #1/(cos(15.2du))=1.04

def callback5(data):
    #rospy.loginfo("depthob2 : %f ", data.data) #right-up
    global depthob2
    depthob2 = data.data
    global dist_ob2
    dist_ob2 = 1.04*depthob2 #1/(cos(15.2du))=1.04

def callback6(data):
    #rospy.loginfo("depthob3 : %f ", data.data) #left-down
    global depthob3
    depthob3 = data.data
    global dist_ob3
    dist_ob3 = 1.04*depthob3 #1/(cos(15.2du))=1.04

    
def callback7(data):
    #rospy.loginfo("depthob4 : %f ", data.data) #left-up
    global depthob4
    depthob4 = data.data
    global dist_ob4
    dist_ob4 = 1.04*depthob4 #1/(cos(15.2du))=1.04
    object_x1 = rospy.Publisher('object_x1', Float64, queue_size=10)#right-down
    object_y1 = rospy.Publisher('object_y1', Float64, queue_size=10)
    object_x2 = rospy.Publisher('object_x2', Float64, queue_size=10)#right-up
    object_y2 = rospy.Publisher('object_y2', Float64, queue_size=10)
    object_x3 = rospy.Publisher('object_x3', Float64, queue_size=10)#left-down
    object_y3 = rospy.Publisher('object_y3', Float64, queue_size=10)
    object_x4 = rospy.Publisher('object_x4', Float64, queue_size=10)#left-up
    object_y4 = rospy.Publisher('object_y4', Float64, queue_size=10)
    rospy.sleep(20)
    f = open("out.txt", "a") 
    global offx1
    offx1 = dist_ob1*(math.cos(angle-0.084*math.pi))
    global objectx1
    objectx1 = posex + offx1
    print("objectx1: " + str(objectx1))
    #print >> f, ("objectx1: " + str(objectx1))
    print >> f, (objectx1)
    object_x1.publish(objectx1)
    global offy1
    offy1 = dist_ob1*(math.sin(angle-0.084*math.pi))
    global objecty1
    objecty1 = posey + offy1
    print("objecty1: " + str(objecty1))
    #print >> f, ("objecty1: " + str(objecty1))
    print >> f, (objecty1)
    object_y1.publish(objecty1)
    global offx2
    offx2 = dist_ob2*(math.cos(angle-0.084*math.pi))
    global objectx2
    objectx2 = posex + offx2
    print("objectx2: " + str(objectx2))
    #print >> f, ("objectx2: " + str(objectx2))
    print >> f, (objectx2)
    object_x2.publish(objectx2)
    global offy2
    offy2 = dist_ob2*(math.sin(angle-0.084*math.pi))
    global objecty2
    objecty2 = posey + offy2
    print("objecty2: " + str(objecty2))
    #print >> f, ("objecty2: " + str(objecty2))
    print >> f, (objecty2)
    object_y2.publish(objecty2)
    global offx3
    offx3 = dist_ob3*(math.cos(angle+0.084*math.pi))
    global objectx3
    objectx3 = posex + offx3
    print("objectx3: " + str(objectx3))
    #print >> f, ("objectx3: " + str(objectx3))
    print >> f, (objectx3)
    object_x3.publish(objectx3)
    global offy3
    offy3 = dist_ob3*(math.sin(angle+0.084*math.pi))
    global objecty3
    objecty3 = posey + offy3
    print("objecty3: " + str(objecty3))
    #print >> f, ("objecty3: " + str(objecty3))
    print >> f, (objecty3)
    object_y3.publish(objecty3)  
    global offx4
    offx4 = dist_ob4*(math.cos(angle+0.084*math.pi))
    global objectx4
    objectx4 = posex + offx4
    print("objectx4: " + str(objectx4))
    #print >> f, ("objectx4: " + str(objectx4))
    print >> f, (objectx4)
    object_x4.publish(objectx4)
    global offy4
    offy4 = dist_ob4*(math.sin(angle+0.084*math.pi))
    global objecty4
    objecty4 = posey + offy4
    print("objecty4: " + str(objecty4))
    #print >> f, ("objecty4: " + str(objecty4))
    print >> f, (objecty4)
    object_y4.publish(objecty4)
    
    file_name = save_image_once.save_image_once()
    #os.system('python save_image_once.py')
    #print(offx)
    print(file_name)
    #print >> f, (file_name + "\n")
    file_name2 = "/home/lyy/semantic_map/" + file_name 
    print >> f, (file_name2 + "\n")
    f.close() 
    f_rcnn = open("/home/lyy/py-faster-rcnn/tools/result_test.txt", "a")
    print >> f_rcnn, ('#############################################@#####') 
    print >> f_rcnn, (file_name2 + "\n")  
    f_rcnn.close()
    shutil.copy(file_name2,'/home/lyy/py-faster-rcnn/data/demo/image.jpg')
    #os.system('python cs.py ' + file_name2 + ' ' + '>>' + ' ' + 'out.txt')      
    print("copy succeeded!")    
    os.system('python /home/lyy/py-faster-rcnn/tools/demo_cs.py --cpu')
    print('okok')

'''
def callback4(data):
    #rospy.loginfo("turtlebot_follower/dist_object : %f ", data.data)
    object_x = rospy.Publisher('object_x', Float64, queue_size=10)
    object_y = rospy.Publisher('object_y', Float64, queue_size=10)
    rospy.sleep(5)
    f = open("out.txt", "a") 
    global depth
    depth = data.data
    global offx
    offx = depth * (math.cos(angle))
    global objectx
    objectx = posex + offx
    print("objectx: " + str(objectx))
    print >> f, ("objectx: " + str(objectx))
    object_x.publish(objectx)
    
    global offy
    offy = depth * (math.sin(angle))
    global objecty
    objecty = posey + offy
    print("objecty: " + str(objecty))
    print >> f, ("objecty: " + str(objecty))
    object_y.publish(objecty)
    
    file_name = save_image_once.save_image_once()
    #os.system('python save_image_once.py')
    #print(offx)
    print(file_name)
    print >> f, (file_name + "\n")
    file_name2 = "/home/lyy/semantic_map/" + file_name
    #os.system('python cs.py ' + file_name2 + ' ' + '>>' + ' ' + 'out.txt')
    print("ok")
    
    f.close()
'''

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/turtlebot_angle", Float64, callback1)
    rospy.Subscriber("/turtlebot_posex", Float64, callback2)
    rospy.Subscriber("/turtlebot_posey", Float64, callback3)
    
    rospy.Subscriber("/turtlebot_follower/dist_object1", Float64, callback4)
    rospy.Subscriber("/turtlebot_follower/dist_object2", Float64, callback5)
    rospy.Subscriber("/turtlebot_follower/dist_object3", Float64, callback6)
    rospy.Subscriber("/turtlebot_follower/dist_object4", Float64, callback7)
    #rospy.Subscriber("/turtlebot_follower/dist_object", Float64, callback8)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



