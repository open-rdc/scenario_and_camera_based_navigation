#!/usr/bin/env python3
from __future__ import print_function

from numpy import dtype
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_with_direction_net_branch_off import *
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
        self.action_num = 1
        self.dl = deep_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_center/image_raw", Image, self.callback)
        self.image_left_sub = rospy.Subscriber("/camera_left/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber("/camera_right/image_raw", Image, self.callback_right_camera)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.action_pub = rospy.Publisher("action", Int8, queue_size=1)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.srv = rospy.Service('/training', SetBool, self.callback_dl_training)
        self.loop_count_srv = rospy.Service('loop_count', SetBool,self.loop_count_callback)
        self.mode_save_srv = rospy.Service('/model_save', Trigger, self.callback_model_save)
        self.pose_sub = rospy.Subscriber("/mcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback_path) 
        self.cmd_dir_sub = rospy.Subscriber("/cmd_dir_intersection", cmd_dir_intersection, self.callback_cmd,queue_size=1)
        self.min_distance = 0.0
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.path_pose = PoseArray()
        self.learning = True
        self.select_dl = False
        self.loop_count_flag = False
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.place = 'cit3f'
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/result_with_dir_' + str(self.mode) + '/'
        self.save_image_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(self.mode) + '/' + str(self.place) + '/' + str(self.start_time) + '/image.pt'
        self.save_dir_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(self.mode) + '/' + str(self.place) + '/' + str(self.start_time) + '/dir.pt'
        self.save_vel_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(self.mode) + '/' + str(self.place) + '/' + str(self.start_time) + '/vel.pt'
        self.save_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/model_with_dir_' + str(self.mode) + '/cit3f/direction/'
        # self.load_path =roslib.packages.get_pkg_dir('nav_cloning') + '/data/model_with_dir_' + str(self.mode) + '/cit3f/direction/1/model.pt'
        self.load_image_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(self.mode) + '/' + str(self.place) + '/' + 'old10000' + '/image' + '/image.pt'
        self.load_dir_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(self.mode) + '/' + str(self.place) + '/' + 'old10000' + '/dir' + '/dir.pt'
        self.load_vel_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(self.mode) + '/' + str(self.place) + '/' + 'old10000' + '/vel' + '/vel.pt'
        self.previous_reset_time = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_the = 0.0
        self.is_started = False
        self.cmd_dir_data = [0, 0, 0]
        # self.episode_num
        # self.target_dataset
        self.train_flag = False
        self.padding_data = 3
        # print(self.episode_num)
        self.start_time_s = rospy.get_time()
        os.makedirs(self.path + self.start_time)

        with open(self.path + self.start_time + '/' + 'training.csv', 'w') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(['step', 'mode', 'loss', 'angle_error(rad)', 'distance(m)', 'x(m)', 'y(m)', 'the(rad)', 'direction'])
        self.tracker_sub = rospy.Subscriber("/tracker", Odometry, self.callback_tracker)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_left_camera(self, data):
        try:
            self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_right_camera(self, data):
        try:
            self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_tracker(self, data):
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y
        rot = data.pose.pose.orientation
        angle = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        self.pos_the = angle[2]

    def callback_path(self, data):
        self.path_pose = data

    def callback_pose(self, data):
        distance_list = []
        pos = data.pose.pose.position
        for pose in self.path_pose.poses:
            path = pose.pose.position
            distance = np.sqrt(abs((pos.x - path.x)**2 + (pos.y - path.y)**2))
            distance_list.append(distance)

        if distance_list:
            self.min_distance = min(distance_list)

    def callback_cmd(self, data):
        self.cmd_dir_data = data.cmd_dir

    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def callback_dl_training(self, data):
        resp = SetBoolResponse()
        self.learning = data.data
        resp.message = "Training: " + str(self.learning)
        resp.success = True
        return resp

    def loop_count_callback(self, data):
        resp = SetBoolResponse()
        self.loop_count_flag = data.data
        resp.message = "count flag"
        resp.success= True
        return resp

    def callback_model_save(self, data):
        model_res = SetBoolResponse()
        self.dl.save(self.save_path)
        model_res.message ="model_save"
        model_res.success = True
        return model_res
    
    def loop(self):
        # if self.episode == 0:
            # self.learning = False
            # self.dl.save(self.save_path)
            # self.dl.load(self.load_path)
            # print("load model",self.load_path)
        # if self.episode == self.episode_num:
        #     self.learning = False
        #     self.dl.save(self.save_path)
        #     #self.dl.load(self.load_path)
        # if self.episode == self.episode_num+20000:
        #     os.system('killall roslaunch')
        #     sys.exit()

        if self.learning:
            dataset = self.dl.load_dataset(self.load_image_path, self.load_dir_path, self.load_vel_path)
            loss = self.dl.trains(dataset)
            self.dl.save(self.save_path)
            print("Finish learning")
            self.learning = False

            print(str(self.episode) + ", training, loss: " + str(loss) + ", angle_error: " + str(angle_error) + ", distance: " + str(distance) + ", self.cmd_dir_data: " + str(self.cmd_dir_data))
            self.episode += 1
            #line = [str(self.episode), "training", str(loss), str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the), str(self.cmd_dir_data)]
            #with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:
            #    writer = csv.writer(f, lineterminator='\n')
            #    writer.writerow(line)

        else:
            print(str(self.episode) + ", test, angular:" + str(target_action) + ", distance: " + str(distance) + ", self.cmd_dir_data: " + str(self.cmd_dir_data))

            self.episode += 1
            angle_error = abs(self.action - target_action)
            line = [str(self.episode), "test", "0", str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the), str(self.cmd_dir_data)]
            with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)
            self.vel.linear.x = 0.2
            self.vel.angular.z = target_action
            self.nav_pub.publish(self.vel)

        # temp = copy.deepcopy(img)
        # cv2.imshow("Resized Image", temp)
        # temp = copy.deepcopy(img_left)
        # cv2.imshow("Resized Left Image", temp)
        # temp = copy.deepcopy(img_right)
        # cv2.imshow("Resized Right Image", temp)
        # cv2.waitKey(1)

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()