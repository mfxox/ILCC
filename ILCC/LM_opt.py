# coding=utf-8
import cPickle
from scipy.optimize import minimize, root
import numpy as np
import transforms3d
import cv2
import os
import math
from scipy.optimize import least_squares
import pyopengv
import time
import config

params = config.default_params()
from ast import literal_eval as make_tuple

(H, W) = make_tuple(params['image_res'])

if params['camera_type'] == "perspective":
    intrinsic_paras_tuple = make_tuple(params['instrinsic_para'])
    intrinsic_paras = np.array(intrinsic_paras_tuple).reshape(3, 3)


# Checks if a matrix is a valid rotation matrix.
# from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def voxel2pixel(ls):
    inclination = ls[0]
    azimuth = ls[1]
    r = H / 2 - int((inclination / np.pi) * H)
    c = int((-azimuth / (2 * np.pi)) * W) + W / 2
    return [c, r]


def xyz2angle(pcd):
    inclination = np.arcsin(pcd[2] / np.linalg.norm(pcd))  # 2017/03/12 为什么是cos？不是sin吗
    # print np.rad2deg(inclination)
    # azimuth = -np.arctan2(pcd[1], pcd[0]) + np.pi / 2
    azimuth = np.arctan2(pcd[1], pcd[0])  # 2017/04/02　p1
    # if azimuth < 0:
    #     azimuth += np.pi
    return [inclination, azimuth]


def convert2_ang_cm(ls):
    ret = []
    for i in xrange(3):
        ret.append(np.rad2deg(ls[i] % 3.14))
    for i in xrange(3):
        ret.append(ls[3 + i] * 100)
    return ret


def convert2_rad_m(ls):
    ret = []
    for i in xrange(3):
        ret.append(np.deg2rad(ls[i]))
    for i in xrange(3):
        ret.append(ls[3 + i] / 100)
    return ret


def calc_inintial_guess(corners_in_img_arr, corners_in_pcd_arr, method="UPNP"):
    # number_of_points_for_initial = int(corners_in_img_arr.shape[0])
    img_bearing_vectors = []
    if params["camera_type"] == "panoramic":
        for pix in corners_in_img_arr:
            angs = pixel2angle(pix)
            img_bearing_vectors.append(
                [np.cos(angs[0]) * np.cos(angs[1]), np.cos(angs[0]) * np.sin(angs[1]), np.sin(angs[0])])
    elif params["camera_type"] == "perspective":
        inv_K = np.linalg.inv(intrinsic_paras)
        tmp_corners_in_img = np.hstack([corners_in_img_arr, 1 + np.zeros(corners_in_img_arr.shape[0]).reshape(-1, 1)])
        for pix in tmp_corners_in_img:
            tmp = np.dot(inv_K, pix.T).T
            img_bearing_vectors.append(tmp / np.linalg.norm(tmp))
    else:
        raise Exception("camera_type define error!")

    img_bearing_vectors = np.array(img_bearing_vectors)
    pcd_bearing_vectors = np.array(corners_in_pcd_arr) / np.linalg.norm(corners_in_pcd_arr, axis=1).reshape(-1, 1)
    #
    # ransac_transformation = pyopengv.relative_pose_ransac(img_bearing_vectors, pcd_bearing_vectors, "NISTER", 0.01,
    #                                                       1000)
    # # ransac_transformation = pyopengv.relative_pose_fivept_kneip(img_bearing_vectors, pcd_bearing_vectors)

    if method == "RANSAC":
        transformation = pyopengv.absolute_pose_ransac(img_bearing_vectors, pcd_bearing_vectors, "UPNP", 0.001, 100000)
    elif method == "EPNP":
        transformation = pyopengv.absolute_pose_epnp(img_bearing_vectors,
                                                     pcd_bearing_vectors)
    elif method == "UPNP":
        transformation = pyopengv.absolute_pose_upnp(img_bearing_vectors,
                                                     pcd_bearing_vectors)[0]
    else:
        raise Exception("Opengv method error!")

    print "initial guess by relative pose: ", transformation
    # print ransac_transformation
    angs = rotationMatrixToEulerAngles(transformation[:3, :3].T).tolist()
    ret = []
    ret.extend(angs)
    # scale = least_squares(cal_scale_cost, x0=np.random.random(),
    #                       args=(img_bearing_vectors, pcd_bearing_vectors, corners_in_pcd_arr, ransac_transformation),
    #                       method="lm", ftol=1e-10, max_nfev=20000)
    # print scale
    # print "estimated scale: ", scale.x
    # print scale.x * ransac_transformation[:3, 3]
    # ret.extend((scale.x * ransac_transformation[:3, 3]).tolist())
    ret.extend((-transformation[:3, 3]).tolist())
    return np.array(ret)


def back_project(r_t, img, corners_in_img_arr, corners_in_pcd_arr):
    if params["camera_type"] == "panoramic":
        S1 = 20
        S2 = 15
        N1 = 10
        N2 = 5
        L = 2
    elif params["camera_type"] == "perspective":
        S1 = 4
        S2 = 3
        N1 = 2
        N2 = 1
        L = 1
    else:
        raise Exception("camera type defined error!")

    c1 = np.zeros([corners_in_img_arr.shape[0], 3]).astype(np.int32) + np.array([0, 255, 0])
    c1[0] = np.array([255, 0, 0])
    c1[-1] = np.array([0, 0, 255])
    s1 = np.zeros(corners_in_img_arr.shape[0]).astype(np.int32) + N1
    s1[0] = S1
    s1[-1] = S1

    c2 = np.zeros([corners_in_pcd_arr.shape[0], 3]).astype(np.int32) + np.array([0, 255, 255])
    c2[0] = np.array([255, 0, 0])
    c2[-1] = np.array([0, 0, 255])
    s2 = np.zeros(corners_in_pcd_arr.shape[0]).astype(np.int32) + N2
    s2[0] = S2
    s2[-1] = S2

    cv2.polylines(img, [corners_in_img_arr], 0, (255, 255, 0), L, lineType=16)
    for i in xrange(corners_in_img_arr.shape[0]):
        cv2.circle(img, tuple(corners_in_img_arr[i].tolist()), s1[i], tuple(c1[i].tolist()), 3)
        cv2.putText(img, str(i + 1), tuple(corners_in_img_arr[i].tolist()), cv2.FONT_HERSHEY_PLAIN, .7,
                    (255, 182, 193), 0)

    transformed_pcd = roate_with_rt(r_t, corners_in_pcd_arr)

    # rot_mat = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], 0),
    #                  np.dot(transforms3d.axangles.axangle2mat([0, 1, 0], r_t[1]),
    #                         transforms3d.axangles.axangle2mat([1, 0, 0], r_t[0])))
    # transformed_pcd = np.dot(rot_mat, corners_in_pcd_arr.T).T + np.array([0, 0, r_t[2]])

    transformed_pcd_ls = transformed_pcd.tolist()
    if params["camera_type"] == "panoramic":
        pcd2angle_s = map(xyz2angle, transformed_pcd_ls)
        proj_corners = np.array(map(voxel2pixel, pcd2angle_s)).astype(np.int32)
    elif params["camera_type"] == "perspective":
        proj_corners = ((np.dot(intrinsic_paras, transformed_pcd.T)).T)
        # proj_corners = proj_corners[:,:2].astype(np.int32)
        proj_corners = (proj_corners / proj_corners[:, 2].reshape(-1, 1))[:, :2].astype(np.int32)
    else:
        raise Exception("Camera_type define error!")
    # print proj_corners

    # cv2.polylines(img, [np.fliplr(proj_corners)], 0, (0, 255, 255), 1, lineType=16)
    cv2.polylines(img, [proj_corners], 0, (0, 255, 255), 1, lineType=16)
    for i in xrange(proj_corners.shape[0]):
        # cv2.circle(img, (proj_corners[i][1], proj_corners[i][0]), s2[i], tuple(c2[i].tolist()), 1)
        # cv2.putText(img, str(i + 1), (proj_corners[i][1], proj_corners[i][0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0),
        #             1)
        cv2.circle(img, (proj_corners[i][0], proj_corners[i][1]), s2[i], tuple(c2[i].tolist()), 1)
        cv2.putText(img, str(i + 1), (proj_corners[i][0], proj_corners[i][1]), cv2.FONT_HERSHEY_SIMPLEX, .3,
                    (255, 165, 0),0)
    return img


def pixel2angle(pix):
    u = pix[1]
    v = pix[0]
    inclination = (H / 2 - u) * np.pi / H
    azimuth = - (v - W / 2) * 2 * np.pi / W
    return [inclination, azimuth]


def roate_with_rt(r_t, arr):  # zhengchang [ang_x ang_y ang_z x y z]# r_t=np.array([ang_z,t_z,ang_x_ang_y,t_x,_t_y])
    rot_mat = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], r_t[2]),
                     np.dot(transforms3d.axangles.axangle2mat([0, 1, 0], r_t[1]),
                            transforms3d.axangles.axangle2mat([1, 0, 0], r_t[0])))

    transformed_pcd = np.dot(rot_mat, arr.T).T + r_t[[3, 4, 5]]
    return transformed_pcd


def cost_func(r_t, corners_in_pcd_arr,
              corners_in_img_arr):  # r_t:theta_x,theta_y,theta_z,t_x,t_y,t_z    corners_in_pcd_arr:n*2  corners_in_img_arr:n*2

    transformed_pcd = roate_with_rt(r_t, corners_in_pcd_arr)

    if params['camera_type'] == "panoramic":
        transformed_pcd_ls = transformed_pcd.tolist()
        corners_in_img_arr_ls = corners_in_img_arr.tolist()
        ang_for_pix_ls = map(pixel2angle, corners_in_img_arr_ls)

        pcd2angle_s = map(xyz2angle, transformed_pcd_ls)

        num = corners_in_pcd_arr.shape[0]
        ls = []
        for i in range(0, num):
            r_cost = abs(np.array(ang_for_pix_ls)[i][0] - np.array(pcd2angle_s)[i][0])
            c_cost = min(abs(np.array(ang_for_pix_ls)[i][1] - np.array(pcd2angle_s)[i][1]),
                         abs(abs(np.array(ang_for_pix_ls)[i][1] - np.array(pcd2angle_s)[i][1]) - np.pi))
            ls.append(np.sqrt(r_cost ** 2 + c_cost ** 2) * 100)
        residuals = np.array(ls)

    elif params['camera_type'] == "perspective":
        cam_coord_pcd = transformed_pcd.copy()
        # coordinate axis swap from world/velodyne coordinate? x→z y→-x z→y
        # cam_coord_pcd[:, 0] = transformed_pcd[:, 2]
        # cam_coord_pcd[:, 1] = -transformed_pcd[:, 0]
        # cam_coord_pcd[:, 2] = transformed_pcd[:, 1]

        # pcd_to_pix = (np.dot(intrinsic_paras, transformed_pcd.T)).T
        pcd_to_pix = (np.dot(intrinsic_paras, cam_coord_pcd.T)).T
        pcd_to_pix = pcd_to_pix / pcd_to_pix[:, 2].reshape(-1, 1)

        # pcd_to_pix = (
        #     cv2.projectPoints(corners_in_pcd_arr, np.zeros(3), np.zeros(3), intrinsic_paras, np.zeros(4))[0]).reshape(
        #     -1, 2)
        # print pcd_to_pix
        num = corners_in_pcd_arr.shape[0]
        ls = []
        for i in range(0, num):
            r_cost = abs(pcd_to_pix[i][0] - corners_in_img_arr[i][0])
            c_cost = abs(pcd_to_pix[i][1] - corners_in_img_arr[i][1])
            # ls.append(np.sqrt(r_cost ** 2 + c_cost ** 2))
            ls.append((abs(r_cost) + abs(c_cost)))
        residuals = np.array(ls)
    else:
        raise Exception("Camera type not correctly defined!")
    return residuals


def cost_func_min(r_t, corners_in_pcd_arr,
                  corners_in_img_arr):  # r_t:theta_x,theta_y,theta_z,t_x,t_y,t_z    corners_in_pcd_arr:n*2  corners_in_img_arr:n*2

    transformed_pcd = roate_with_rt(r_t, corners_in_pcd_arr)

    transformed_pcd_ls = transformed_pcd.tolist()

    corners_in_img_arr_ls = corners_in_img_arr.tolist()
    ang_for_pix_ls = map(pixel2angle, corners_in_img_arr_ls)

    pcd2angle_s = map(xyz2angle, transformed_pcd_ls)

    cost = np.linalg.norm(np.array(ang_for_pix_ls) - np.array(pcd2angle_s)) * 10
    return cost


def run_min(args, initial_guess):  # (np.random.random(6)).tolist()
    print
    res = least_squares(cost_func, initial_guess, args=args, method="lm", ftol=1e-15, max_nfev=100000)  # 1e-10

    # with bounds limitation
    # res = least_squares(cost_func, initial_guess, args=args, method="trf", ftol=1e-15, max_nfev=1000000,
    #                     bounds=([-np.pi, -np.pi, -np.pi, -2, -2, -2], [np.pi, np.pi, np.pi, 2, 2, 2]))  # 1e-10
    return res


def opt_r_t(corners_in_img_arr, corners_in_pcd_arr, initial_guess=np.zeros(6).tolist(), save_backproj=False,
            imgfile=None, pkl_path=None):
    if save_backproj:
        pkl_file = pkl_path[0].split("proposed/")[-1]
        with open(pkl_file, "r") as f:
            ls = cPickle.load(f)
        pcd_ls = []
        for seg in ls:
            pcd_ls.extend(seg)
        pcd_ls_arr = np.array(pcd_ls)

    args = (corners_in_pcd_arr, corners_in_img_arr)

    res = run_min(args, initial_guess)  # , initial_guess=(np.random.random(6) * 1.5).tolist()

    if save_backproj:
        img = cv2.imread(imgfile)
        img_ret = back_project(res.x, img, corners_in_img_arr, corners_in_pcd_arr)
        save_file_name = (imgfile.split(".")[0]) + "_backproj." + params['image_format']
        cv2.imwrite(save_file_name, img_ret)
        # print type(res.fun)

    return res


def cal_ext_paras():
    ls = (np.arange(1, 5)).tolist()
    # res_ls = []
    # pnp_ls = []

    corners_in_image_all = np.array([]).reshape(0, 2)
    corners_in_pcd_all = np.array([]).reshape(0, 3)

    if params['camera_type'] == "panoramic":
        print "Optimize the extrinsic parameters with panoramic model."
        for i in ls:
            imgfile = "img/" + str(i).zfill(4) + "." + params['image_format']
            cords_file = "output/img_corners/" + str(i).zfill(4) + "_img_corners.txt"
            corners_in_img_arr = np.genfromtxt(cords_file, delimiter=",").astype(np.int32)
            # make sure the corners are counted start from left lower
            if np.linalg.norm(np.array(corners_in_img_arr[0]) - np.array([0, H])) > np.linalg.norm(
                    np.array(corners_in_img_arr[-1]) - np.array([0, H])):
                print imgfile + " is counted in reversed order"
                corners_in_img_arr = np.flipud(corners_in_img_arr)

            pcd_result_file = "output/pcd_seg/" + str(i).zfill(4) + "_pcd_result.pkl"
            # print imgfile
            with open(os.path.abspath(pcd_result_file), "r") as f:
                pcd_result_ls = cPickle.load(f)
            assert pcd_result_ls is not None

            corner_arr = pcd_result_ls[4].reshape(-1, 2)

            num = corner_arr.shape[0]
            corner_arr = np.hstack([corner_arr, np.zeros(num).reshape(num, 1)])
            # print corner_arr.shape
            rot1 = pcd_result_ls[0]
            t1 = pcd_result_ls[1].reshape(1, 3)
            rot2 = pcd_result_ls[2]
            t2 = pcd_result_ls[3].reshape(1, 3)

            corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)

            corners_in_image_all = np.vstack([corners_in_image_all, corners_in_img_arr])
            corners_in_pcd_all = np.vstack([corners_in_pcd_all, corners_in_pcd_arr])
        # print "corners_in_pcd_all num of rows: ", corners_in_pcd_all.shape

        try:
            initial_guess = calc_inintial_guess(corners_in_image_all.copy(), corners_in_pcd_all.copy(),
                                                method="UPNP")
        except:
            upnp_cost = np.inf
            print "upnp can not be applied"

        res = opt_r_t(corners_in_image_all, corners_in_pcd_all, initial_guess=initial_guess,
                      save_backproj=False, )  # initial_guess=initial_guess,
        # res_ls.append(convert2_ang_cm(res.x))
        # pnp_ls.append(initial_guess)

        print "initial guess by UPnP:", initial_guess
        print
        print "final extrinsic parameters:"
        cal_file_name = time.strftime("%Y%m%d_%H%M%S_cali_result.txt")
        np.savetxt(cal_file_name, res.x, delimiter=',')
        print "refined by LM : ", res.x, " unit: [rad,rad,rad,m,m,m]. The result is Saved to ", cal_file_name
        print "unit converted : ", convert2_ang_cm(res.x), "unit: [deg,deg,deg,cm,cm,cm]"
        print
        print
        print
        # print "The difference of relative_pose - res.x: ", initial_guess - res.x
        # np.savetxt('intes_result', res_ls, delimiter=',')
        # np.savetxt('pnp_result', pnp_ls, delimiter=',')

        # back_proj = True
        if params['back_proj_corners']:
            # for i in range(start, end):
            for i in ls:
                imgfile = "img/" + str(i).zfill(4) + "." + params['image_format']
                cords_file = "output/img_corners/" + str(i).zfill(4) + "_img_corners.txt"
                corners_in_img_arr = np.genfromtxt(cords_file, delimiter=",").astype(np.int32)
                # make sure the corners are counted start from left lower
                if np.linalg.norm(np.array(corners_in_img_arr[0]) - np.array([0, H])) > np.linalg.norm(
                        np.array(corners_in_img_arr[-1]) - np.array([0, H])):
                    corners_in_img_arr = np.flipud(corners_in_img_arr)

                pcd_result_file = "output/pcd_seg/" + str(i).zfill(4) + "_pcd_result.pkl"
                # print imgfile
                with open(os.path.abspath(pcd_result_file), "r") as f:
                    pcd_result_ls = cPickle.load(f)
                assert pcd_result_ls is not None

                corner_arr = pcd_result_ls[4].reshape(-1, 2)

                num = corner_arr.shape[0]
                corner_arr = np.hstack([corner_arr, np.zeros(num).reshape(num, 1)])
                rot1 = pcd_result_ls[0]
                t1 = pcd_result_ls[1].reshape(1, 3)
                rot2 = pcd_result_ls[2]
                t2 = pcd_result_ls[3].reshape(1, 3)
                corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)
                ret = back_project(res.x, cv2.imread(imgfile), corners_in_img_arr, corners_in_pcd_arr)
                save_file = "output/" + str(i).zfill(4) + "_cal_backproj." + params['image_format']
                cv2.imwrite(save_file, ret)
    elif params['camera_type'] == "perspective":
        print "Optimize the extrinsic parameters with perspective model."
        for i in ls:
            imgfile = "img/" + str(i).zfill(4) + "." + params['image_format']
            cords_file = "output/img_corners/" + str(i).zfill(4) + "_img_corners.txt"
            corners_in_img_arr = np.genfromtxt(cords_file, delimiter=",").astype(np.int32)
            # make sure the corners are counted start from left lower
            if np.linalg.norm(np.array(corners_in_img_arr[0]) - np.array([0, H])) > np.linalg.norm(
                    np.array(corners_in_img_arr[-1]) - np.array([0, H])):
                print imgfile + " is counted in reversed order"
                corners_in_img_arr = np.flipud(corners_in_img_arr)

            pcd_result_file = "output/pcd_seg/" + str(i).zfill(4) + "_pcd_result.pkl"
            # print imgfile
            with open(os.path.abspath(pcd_result_file), "r") as f:
                pcd_result_ls = cPickle.load(f)
            assert pcd_result_ls is not None

            corner_arr = pcd_result_ls[4].reshape(-1, 2)

            num = corner_arr.shape[0]
            corner_arr = np.hstack([corner_arr, np.zeros(num).reshape(num, 1)])
            # print corner_arr.shape
            rot1 = pcd_result_ls[0]
            t1 = pcd_result_ls[1].reshape(1, 3)
            rot2 = pcd_result_ls[2]
            t2 = pcd_result_ls[3].reshape(1, 3)

            corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)

            corners_in_image_all = np.vstack([corners_in_image_all, corners_in_img_arr])
            corners_in_pcd_all = np.vstack([corners_in_pcd_all, corners_in_pcd_arr])
        # print "corners_in_pcd_all num of rows: ", corners_in_pcd_all.shape

        try:
            initial_guess = calc_inintial_guess(corners_in_image_all.copy(), corners_in_pcd_all.copy(),
                                                method="UPNP")
        except:
            upnp_cost = np.inf
            initial_guess = np.zeros(6).tolist()
            print "upnp can not be applied"

        res = opt_r_t(corners_in_image_all, corners_in_pcd_all, initial_guess=initial_guess,
                      save_backproj=False, )  # initial_guess=initial_guess,
        print res.cost
        # res_ls.append(convert2_ang_cm(res.x))
        # pnp_ls.append(initial_guess)

        print "initial guess by UPnP:", initial_guess
        print
        print "final extrinsic parameters:"
        cal_file_name = time.strftime("%Y%m%d_%H%M%S_cali_result.txt")
        np.savetxt(cal_file_name, res.x, delimiter=',')
        print "refined by LM : ", res.x, " unit: [rad,rad,rad,m,m,m]. The result is Saved to ", cal_file_name
        print "unit converted : ", convert2_ang_cm(res.x), "unit: [deg,deg,deg,cm,cm,cm]"
        print
        print
        print
        # print "The difference of relative_pose - res.x: ", initial_guess - res.x
        # np.savetxt('intes_result', res_ls, delimiter=',')
        # np.savetxt('pnp_result', pnp_ls, delimiter=',')

        # back_proj = True
        if params['back_proj_corners']:
            # for i in range(start, end):
            for i in ls:
                imgfile = "img/" + str(i).zfill(4) + "." + params['image_format']
                cords_file = "output/img_corners/" + str(i).zfill(4) + "_img_corners.txt"
                corners_in_img_arr = np.genfromtxt(cords_file, delimiter=",").astype(np.int32)
                # make sure the corners are counted start from left lower
                if np.linalg.norm(np.array(corners_in_img_arr[0]) - np.array([0, H])) > np.linalg.norm(
                        np.array(corners_in_img_arr[-1]) - np.array([0, H])):
                    corners_in_img_arr = np.flipud(corners_in_img_arr)

                pcd_result_file = "output/pcd_seg/" + str(i).zfill(4) + "_pcd_result.pkl"
                # print imgfile
                with open(os.path.abspath(pcd_result_file), "r") as f:
                    pcd_result_ls = cPickle.load(f)
                assert pcd_result_ls is not None

                corner_arr = pcd_result_ls[4].reshape(-1, 2)

                num = corner_arr.shape[0]
                corner_arr = np.hstack([corner_arr, np.zeros(num).reshape(num, 1)])
                rot1 = pcd_result_ls[0]
                t1 = pcd_result_ls[1].reshape(1, 3)
                rot2 = pcd_result_ls[2]
                t2 = pcd_result_ls[3].reshape(1, 3)
                corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)
                ret = back_project(res.x, cv2.imread(imgfile), corners_in_img_arr, corners_in_pcd_arr)
                # ret = back_project(initial_guess, cv2.imread(imgfile), corners_in_img_arr, corners_in_pcd_arr)
                save_file = "output/" + str(i).zfill(4) + "_cal_backproj." + params['image_format']
                cv2.imwrite(save_file, ret)

    else:
        raise "The input camera type is not implemented yet!"


if __name__ == "__main__":
    cal_ext_paras()
