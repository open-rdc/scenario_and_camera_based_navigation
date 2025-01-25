import csv
import numpy as np
import glob
import os

# Ensure lists is populated with file paths
lists = []
experiment = 'branch_375_cross'

for filename in sorted(glob.glob('/home/takumi/catkin_ws/src/nav_cloning/data/result_with_dir_selected_training/' + experiment + '/*/training.csv'), key=lambda f: os.stat(f).st_mtime, reverse=True):
    lists.append(filename)

def average calculator():
    index = 0
    file_number = 1

    # Create lists to store all distances and angle_errors across files
    all_distances = []
    all_angle_errors = []

    while index < 100 and index < len(lists):
        distances = []
        angle_errors = []
        
        with open(lists[index]) as f:
            for row in csv.reader(f):
                str_step, mode, nazo, angle_error, distance, str_x, str_y, str_the, cmd_dir = row
                if mode == "test":
                    distance_value = float(distance)
                    angle_error_value = float(angle_error)

                    # Only append values where distance is less than 1
                    if distance_value < 1:
                        distances.append(distance_value)
                        angle_errors.append(angle_error_value)
        
        # Add current file's distances and angle errors to the global lists
        all_distances.extend(distances)
        all_angle_errors.extend(angle_errors)

        # Calculate the averages for the current file
        if distances:
            avg_distance = np.mean(distances)
        else:
            avg_distance = 0

        if angle_errors:
            avg_angle_error = np.mean(angle_errors)
        else:
            avg_angle_error = 0

        print(f"File {file_number} - Average Distance: {avg_distance}, Average Angle Error: {avg_angle_error}")

        index += 1
        file_number += 1

    # Calculate the overall averages across all files
    if all_distances:
        overall_avg_distance = np.mean(all_distances)
    else:
        overall_avg_distance = 0

    if all_angle_errors:
        overall_avg_angle_error = np.mean(all_angle_errors)
    else:
        overall_avg_angle_error = 0

    print(f"Overall Average Distance: {overall_avg_distance}, Overall Average Angle Error: {overall_avg_angle_error}")

if __name__ == '__main__':
    average_calculator()
