#!/usr/bin/env python3
from __future__ import print_function

from numpy import dtype
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from network import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from std_msgs.msg import Int8MultiArray
from scenario_navigation_msgs.msg import cmd_dir_intersection
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse
import csv
import os
import time
import copy
import sys
import tf
from nav_msgs.msg import Odometry

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "selected_training")
        self.dl = deep_learning()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_center/image_raw", Image, self.callback)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.nav_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
        self.cmd_dir_sub = rospy.Subscriber("/cmd_dir_intersection", cmd_dir_intersection, self.callback_cmd,queue_size=1)
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.learning = False
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/model/demo/model.pt'

        self.cmd_dir_data = (0, 0, 0)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_cmd(self, data):
        self.cmd_dir_data = data.cmd_dir
    
    def loop(self):
        # print('\033[32m'+'test_mode'+'\033[0m')
        if self.cv_image.size != 640 * 480 * 3:
            return
        img = resize(self.cv_image, (48, 64), mode='constant')
        ros_time = str(rospy.Time.now())

        if self.episode == 0:
            self.learning = False
            self.dl.load(self.load_path)
            print("load model",self.load_path)
        
        print('\033[32m'+'test_mode'+'\033[0m')
        # stop
        if self.cmd_dir_data == (0, 0, 0):
            print('\033[32m'+'stop'+'\033[0m')
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            target_action = 0.0
        else:
            print('\033[32m'+'move'+'\033[0m')
            target_action = self.dl.act(img, self.cmd_dir_data)
            self.vel.linear.x = 0.2
            self.vel.angular.z = target_action
            # if abs(target_action) >1.82:
            #     target_action=1.82
            # else:
            #     pass

        print(str(self.episode) + ", test, angular:" + str(target_action) + ", self.cmd_dir_data: " + str(self.cmd_dir_data))

        self.episode += 1
        self.nav_pub.publish(self.vel)

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
