#!/usr/bin/env python3

import roslib
roslib.load_manifest('intersection_detector')
import roslib.packages
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from network import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8,String
from std_srvs.srv import Trigger
from std_msgs.msg import Int8MultiArray
from scenario_navigation_msgs.msg import cmd_dir_intersection
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse
import os
import time
import sys
import tf
import glob
from nav_msgs.msg import Odometry

class intersection_detector_node:
    def __init__(self):
        rospy.init_node('intersection_detector_node', anonymous=True)
        self.dl = deep_learning()
        self.bridge = CvBridge()
        self.intersection_pub = rospy.Publisher("passage_type",cmd_dir_intersection,queue_size=1)
        self.image_sub = rospy.Subscriber("/camera_center/image_raw", Image, self.callback)

        self.srv = rospy.Service('/training_intersection', SetBool, self.callback_loop_count)

        self.loop_srv = rospy.Service('/loop_count', SetBool, self.callback_loop_count)
        
        self.cmd_dir_sub = rospy.Subscriber("/cmd_dir_intersection", cmd_dir_intersection, self.callback_cmd,queue_size=1)
        self.min_distance = 0.0
        self.action = 0.0
        self.episode = 0
        self.intersection = cmd_dir_intersection()
        self.path_pose = PoseArray()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.cv_left_image = np.zeros((480,640,3), np.uint8)
        self.cv_right_image = np.zeros((480,640,3), np.uint8)
        self.learning = True

        self.loop_count_flag = False
        self.cat_tensor_flag = False
        self.learning_tensor_flag = False
        self.select_dl = False
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.save_image_path = roslib.packages.get_pkg_dir('intersection_detector') + '/data/dataset/' + str(self.start_time) + '/image/'
        self.save_label_path = roslib.packages.get_pkg_dir('intersection_detector') + '/data/dataset/' + str(self.start_time) + '/label/'
        self.save_path = roslib.packages.get_pkg_dir('intersection_detector') + '/data/model/'
        
        self.previous_reset_time = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_the = 0.0
        self.is_started = False
        self.cmd_dir_data = [0,0,0,0,0,0,0,0]
        self.intersection_list = ["straight_road","dead_end","corner_right","corner_left","cross_road","3_way_right","3_way_center","3_way_left"]
        self.start_time_s = rospy.get_time()

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)


    def callback_cmd(self, data):
        self.cmd_dir_data = data.intersection_label


    def callback_loop_count(self, data):
        resp = SetBoolResponse()
        self.loop_count_flag = data.data
        resp.message = "Training: " + str(self.learning)
        resp.success = True
        return resp


    def loop(self):
        if self.cv_image.size != 640 * 480 * 3:
            return
        # if self.cv_left_image.size != 640 * 480 * 3:
        #     return
        # if self.cv_right_image.size != 640 * 480 * 3:
        #     return
        img = resize(self.cv_image, (48, 64), mode='constant')

        # img_left = resize(self.cv_left_image, (48, 64), mode='constant')
        # img_right = resize(self.cv_right_image, (48, 64), mode='constant')

        ros_time = str(rospy.Time.now())
        
        image_tensor ,label_tensor =self.dl.make_dataset(img,self.cmd_dir_data)

        if self.loop_count_flag:
            self.dl.save_tensor(image_tensor, self.save_image_path, '/image.pt')
            self.dl.save_tensor(label_tensor, self.save_label_path, '/label.pt')
            _, _ = self.dl.training(image_tensor, label_tensor, False)
            self.dl.save(self.save_path)
            self.loop_count_flag = False
            print("Finish learning")
            os.system('killall roslaunch')
            sys.exit()
        else :
            pass

if __name__ == '__main__':
    rg = intersection_detector_node()
    r = rospy.Rate(8.0)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
