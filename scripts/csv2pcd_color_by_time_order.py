# coding=utf-8
'''
Created on 10/3/2017 2:19 07PM Wang Weimin

@author: wangwm
'''

import numpy as np
import sys

laser_beam_num=64
def convert_arr_2_colored_pcd(xyzrgb_arr, save_path):
    rgb_pcd_header = ["# .PCD v.7 - Point Cloud Data file format", "VERSION .7", "FIELDS x y z rgb", "SIZE 4 4 4 4",
                      "TYPE F F F U", "COUNT 1 1 1 1"]
    all_points_num = len(xyzrgb_arr)
    print "all_points_num", all_points_num
    rgb_pcd_header.append("WIDTH " + str(all_points_num))
    rgb_pcd_header.append("HEIGHT 1")
    rgb_pcd_header.append("VIEWPOINT 0 0 0 1 0 0 0")
    rgb_pcd_header.append("POINTS " + str(all_points_num))
    rgb_pcd_header.append("DATA ascii")

    # save_path = "new_xyzrgb.pcd"
    f = open(save_path, "w")
    for line in rgb_pcd_header:
        f.write(line + "\n")
    for line in xyzrgb_arr:
        # color = np.array([0, 255, 0])
        # rgb_value = color[2] << 16 | color[1s] << 8 | color[0]

        color = line[3:].astype(np.uint32)
        rgb_value = color[0] << 16 | color[1] << 8 | color[2]
        f.write('{0:.4f}'.format(line[0]) + " " + '{0:.4f}'.format(line[1]) + " " + '{0:.4f}'.format(
            line[2]) + " " + str(rgb_value) + "\n")
    f.close()
    print "converted"


csv_file = sys.argv[1]
save_path = sys.argv[2]
arr = np.genfromtxt(csv_file, skip_header=1, delimiter=",")[:, :5]

new_arr_ls = []

color_bar = np.random.randint(0, 255, [128, 3])

for m in xrange(laser_beam_num):
    order = np.where(arr[:, 4] == m)  # the 4-th column means the laser id
    temp_data = arr[order[0]][:, :3]  # exact XYZ data with laser_id=i
    temp_point_num = temp_data.shape[0]
    for p in xrange(temp_point_num):
        color = np.array([int(float(p) / temp_point_num * 255), 0, 255 - int(float(p) / temp_point_num * 255)])
        # print color
        new_arr_ls.append(np.hstack([temp_data[p, :3], color]))

convert_arr_2_colored_pcd(new_arr_ls, save_path)
