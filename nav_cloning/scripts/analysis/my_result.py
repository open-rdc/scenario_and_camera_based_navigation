#!/usr/bin/env python3
import csv
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from matplotlib import pyplot
import numpy as np
from PIL import Image
import glob
import os
import roslib
lists = []

experiment = 'route_8/2'

for filename in sorted(glob.glob('/home/takumi/catkin_ws/src/nav_cloning/data/result_with_dir_selected_training/' + experiment + '/*/training.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
# for filename in sorted(glob.glob('/home/yuzuki/ws/master_model_evaluation_ws/src/nav_cloning/data/result_use_dl_output/*/training_all.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
# for filename in sorted(glob.glob('/home/yuzuki/ws/master_model_evaluation_ws/src/nav_cloning/data/result_follow_line/*/training_all.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
# for filename in sorted(glob.glob('/home/yuzuki/result/result_ochi/*/training_all.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
    lists.append(filename)

# def draw_training_pos_willow():
#     index = 0
#     file_number = 1
#     while index < 100:
#         image = Image.open(('/home/takumi/catkin_ws/src/nav_cloning')+'/maps/map.png').convert("L")
#         arr = np.asarray(image)
#         fig = plt.figure()
#         ax = fig.add_subplot(111)
#         ax.imshow(arr, cmap='gray', extent=[-10,50,-10,50])
#         vel = 0.2
#         arrow_dict = dict(arrowstyle = "->", color = "black")
#         count = 0
        
#         print(file_number)
#         with open(lists[index]) as f:
#             for row in csv.reader(f):
#                     str_step, mode, nazo, angle_error, distance, str_x, str_y, str_the, cmd_dir = row
#                     if mode == "test":
#                         x, y = float(str_x), float(str_y)
#                         patch = Circle(xy=(x, y), radius=0.08, facecolor="red") 
#                         ax.add_patch(patch)
#             else:
#                         pass
                
#         ax.set_xlim([-5, 30])
#         ax.set_ylim([-5, 15])
#         plt.show(block=False)
#         pictures_dir = '/home/takumi/catkin_ws/src/nav_cloning/data/pictures/' + experiment
#         if not os.path.exists(pictures_dir):
#             os.makedirs(pictures_dir)
#         plt.savefig(pictures_dir + "/" + str(file_number) + ".png")
#         plt.pause(1)
#         plt.close()

#         index += 1
#         file_number += 1


def draw_training_pos_tsudanuma():
    index = 0
    file_number = 1
    while index < 10000:
        image = Image.open(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/real_tsudanuma2-3_v2.pgm').convert("L")
        image = image.crop((1762, 1253, 2059, 2252))
        arr = np.asarray(image)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.imshow(arr, cmap='gray', extent=[-12.25,3,-12.5,37.2])
        vel = 0.2
        arrow_dict = dict(arrowstyle = "->", color = "black")
        count = 0

        print(file_number)
        with open(lists[index]) as f:
            for row in csv.reader(f):
                    str_step, mode, nazo, angle_error, distance, str_x, str_y, str_the, cmd_dir = row
                    if mode == "test":
                        x, y = float(str_x), float(str_y)
                        patch = Circle(xy=(-x-5, -y+7.7), radius=0.08, facecolor="red") 
                        ax.add_patch(patch)
            else:
                        pass
        
        ax.set_xlim([-13, 3])
        ax.set_ylim([-10, 35]) 
        plt.show(block=False)
        pictures_dir = '/home/takumi/catkin_ws/src/nav_cloning/data/pictures/' + experiment
        if not os.path.exists(pictures_dir):
            os.makedirs(pictures_dir)
        plt.savefig(pictures_dir + "/" + str(file_number) + ".png")
        plt.pause(1)
        plt.close()

        index += 1
        file_number += 1

# def draw_training_pos_oldtsudanuma():
#     index = 0
#     file_number = 1
#     while index < 10000:
#         image = Image.open(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/cit_3f_map.pgm').convert("L")
#         image = image.crop((1762, 1253, 2059, 2252))
#         arr = np.asarray(image)
#         fig = plt.figure()
#         ax = fig.add_subplot(111)
#         ax.imshow(arr, cmap='gray', extent=[-12.25,3,-12.5,37.2])
#         vel = 0.2
#         arrow_dict = dict(arrowstyle = "->", color = "black")
#         count = 0

#         print(file_number)
#         with open(lists[index]) as f:
#             for row in csv.reader(f):
#                     str_step, mode, nazo, angle_error, distance, str_x, str_y, str_the, cmd_dir = row
#                     if mode == "test":
#                         x, y = float(str_x), float(str_y)
#                         patch = Circle(xy=(x, y), radius=0.08, facecolor="red") 
#                         ax.add_patch(patch)
#             else:
#                         pass
        
#         ax.set_xlim([-13, 3])
#         ax.set_ylim([-10, 35]) 
#         plt.show(block=False)
#         pictures_dir = '/home/takumi/catkin_ws/src/nav_cloning/data/pictures/' + experiment
#         if not os.path.exists(pictures_dir):
#             os.makedirs(pictures_dir)
#         plt.savefig(pictures_dir + "/" + str(file_number) + ".png")
#         plt.pause(1)
#         plt.close()

#         index += 1
#         file_number += 1

if __name__ == '__main__':
    # draw_training_pos_willow()
    draw_training_pos_tsudanuma()
    # draw_training_pos_oldtsudanuma():



