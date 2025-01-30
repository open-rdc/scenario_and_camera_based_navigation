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
# from waypoint_nav.msg import cmd_dir_intersection
from scenario_navigation_msgs.msg import cmd_dir_intersection
# from geometry_msgs.msg import PoseWithCovarianceStamped
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
        self.class_num = 8
        self.dl = deep_learning()
        self.bridge = CvBridge()
        # self.intersection_pub = rospy.Publisher("passage_type",String,queue_size=1)
        self.intersection_pub = rospy.Publisher("passage_type",cmd_dir_intersection,queue_size=1)
        self.image_sub = rospy.Subscriber("/camera_center/image_raw", Image, self.callback)

        self.srv = rospy.Service('/training_intersection', SetBool, self.callback_save_tensor)

        self.loop_srv = rospy.Service('/loop_count', SetBool, self.callback_save_tensor)
        
        # self.mode_save_srv = rospy.Service('/model_save_intersection', Trigger, self.callback_model_save)
        self.cmd_dir_sub = rospy.Subscriber("/cmd_dir_intersection", cmd_dir_intersection, self.callback_cmd,queue_size=1)
        self.min_distance = 0.0
        self.action = 0.0
        self.episode = 0
        # self.intersection =String()
        self.intersection = cmd_dir_intersection()
        self.path_pose = PoseArray()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.cv_left_image = np.zeros((480,640,3), np.uint8)
        self.cv_right_image = np.zeros((480,640,3), np.uint8)
        self.learning = True

        self.save_tensor_flag = False
        self.cat_tensor_flag = False
        self.learning_tensor_flag = False
        self.select_dl = False
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.save_image_path = roslib.packages.get_pkg_dir('intersection_detector') + '/../data/intersection_detector/dataset/image/'
        self.save_label_path = roslib.packages.get_pkg_dir('intersection_detector') + '/../data/intersection_detector/dataset/label/'
        self.save_path = roslib.packages.get_pkg_dir('intersection_detector') + '/../data/intersection_detector/model/'
        
        self.previous_reset_time = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_the = 0.0
        self.is_started = False
        self.cmd_dir_data = [0,0,0,0,0,0,0,0]
        self.intersection_list = ["straight_road","dead_end","corner_right","corner_left","cross_road","3_way_right","3_way_center","3_way_left"]
        self.start_time_s = rospy.get_time()
        # os.makedirs(self.path + self.start_time)

    def callback(self, data):
        try:
            # self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)


    def callback_cmd(self, data):
        self.cmd_dir_data = data.intersection_label


    def callback_save_tensor(self, data):
        resp = SetBoolResponse()
        # self.learning = data.data
        self.save_tensor_flag = data.data
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
        # img = resize(self.cv_image, (224, 224), mode='constant')
        
        # rospy.loginfo("start")
        # r, g, b = cv2.split(img)
        # img = np.asanyarray([r,g,b])

        # img_left = resize(self.cv_left_image, (48, 64), mode='constant')
        #r, g, b = cv2.split(img_left)
        #img_left = np.asanyarray([r,g,b])

        # img_right = resize(self.cv_right_image, (48, 64), mode='constant')
        #r, g, b = cv2.split(img_right)
        #img_right = np.asanyarray([r,g,b])
        # cmd_dir = np.asanyarray(self.cmd_dir_data)
        ros_time = str(rospy.Time.now())

        
        # dataset ,dataset_num,train_dataset =self.dl.make_dataset(img,self.cmd_dir_data)
        image_tensor ,label_tensor =self.dl.make_dataset(img,self.cmd_dir_data)
        # intersection, loss = self.dl.act_and_trains(img , self.cmd_dir_data)
        # intersection_left,loss_left = self.dl.act_and_trains(img_left,self.cmd_dir_data)
        # intersection_right , loss_right = self.dl.act_and_trains(img_right, self.cmd_dir_data)
                # end mode
        # intersection_name = self.intersection_list[intersection]
        # ans_intersection =self.intersection_list[self.cmd_dir_data.index(max(self.cmd_dir_data))]
        # self.intersection.intersection_name = self.intersection_list[intersection]
        # self.intersection_pub.publish(self.intersection)
        #print("learning: " + str(self.episode) + ", loss: " + str(loss) + ", label: " + str(intersection) + " , intersection_name: " + str(intersection_name)+" , answer_name: " + str(ans_intersection))
        # print("learning: " + str(self.episode) + ", loss: " + str(loss) + ", label: " + str(intersection) + " , intersection_name: " + str(intersection_name) +", correct label: " + str(self.cmd_dir_data))
        # self.episode += 1
        # line = [str(self.episode), "training", str(loss), str(angle_error), str(distance), str(self.pos_x), str(self.pos_y), str(self.pos_the), str(self.cmd_dir_data)]
        # with open(self.path + self.start_time + '/' + 'training.csv', 'a') as f:
        #     writer = csv.writer(f, lineterminator='\n')
        #     writer.writerow(line)
        if self.save_tensor_flag:
            # dataset ,dataset_num,train_dataset =self.dl.make_dataset(img,self.cmd_dir_data)
            # self.dl.training(train_dataset)
            image_tensor ,label_tensor=self.dl.make_dataset(img,self.cmd_dir_data)
            self.dl.save_bagfile(image_tensor,self.save_image_path,'/image.pt')
            self.dl.save_bagfile(label_tensor,self.save_label_path, '/label.pt')
            self.save_tensor_flag = False
            print(self.save_image_path)
            print(self.save_label_path)
            self.learning_tensor_flag = True
        else :
            pass

        if self.learning_tensor_flag:
            _,_ = self.dl.cat_training(image_tensor, label_tensor, False)
            # _,_ = self.dl.training(self.load_image_path,self.load_label_path)
            # self.dl.save_bagfile(x_tensor,self.save_image_path,'/image.pt')
            # self.dl.save_bagfile(t_tensor,self.save_label_path, '/label.pt')
            self.dl.save(self.save_path)
            self.learning_tensor_flag = False
            print("Finish learning")
            os.system('killall roslaunch')
            sys.exit()
        else:
            pass

if __name__ == '__main__':
    rg = intersection_detector_node()
    # DURATION = 0.1
    # r = rospy.Rate(1 / DURATION)
    # r= rospy.Rate(5.0)
    r = rospy.Rate(8.0)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
