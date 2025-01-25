#!/usr/bin/env python3
import csv
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from PIL import Image
import glob
import os
lists = []

experiment = 'direction_6000_cross'

for filename in sorted(glob.glob('/home/takumi/catkin_ws/src/nav_cloning/data/result_with_dir_selected_training/' + experiment + '/*/training.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
# for filename in sorted(glob.glob('/home/yuzuki/ws/master_model_evaluation_ws/src/nav_cloning/data/result_use_dl_output/*/training_all.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
# for filename in sorted(glob.glob('/home/yuzuki/ws/master_model_evaluation_ws/src/nav_cloning/data/result_follow_line/*/training_all.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
# for filename in sorted(glob.glob('/home/yuzuki/result/result_ochi/*/training_all.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
    lists.append(filename)

def draw_training_pos_cross():
    index = 0
    file_number = 1
    while index < 100:
        image = Image.open(('/home/takumi/catkin_ws/src/nav_cloning')+'/maps/cross_road.pgm').convert("L")
        arr = np.asarray(image)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.imshow(arr, cmap='gray', extent=[-97,97,-97,97])
        vel = 0.2
        arrow_dict = dict(arrowstyle = "->", color = "black")
        count = 0
        
        print(file_number)
        with open(lists[index]) as f:
            for row in csv.reader(f):
                    str_step, mode, nazo, angle_error, distance, str_x, str_y, str_the, cmd_dir = row
                    if mode == "test":
                        x, y = float(str_x), float(str_y)
                        patch = Circle(xy=(x, y), radius=0.08, facecolor="red") 
                        ax.add_patch(patch)
            else:
                        pass
                
        ax.set_xlim([-4, 10])
        ax.set_ylim([-4, 4])
        plt.show(block=False)
        pictures_dir = '/home/takumi/catkin_ws/src/nav_cloning/data/pictures/' + experiment
        if not os.path.exists(pictures_dir):
            os.makedirs(pictures_dir)
        plt.savefig(pictures_dir + "/" + str(file_number) + ".png")
        plt.pause(1)
        plt.close()

        index += 1
        file_number += 1

if __name__ == '__main__':
    draw_training_pos_cross()
   


