import numpy as np
from numpy import linalg as LA
from numpy.linalg import norm, inv
import shutil, copy
from sklearn.decomposition import PCA
from scipy import spatial
import matplotlib.path as mplPath
import transforms3d
from scipy.optimize import minimize
import cPickle
import os
from ast import literal_eval as make_tuple
from multiprocessing import Pool
import re
import warnings
from scipy import stats

import config

params = config.default_params()

# marker length of long side and short side
marker_size = make_tuple(params["pattern_size"])
marker_l = params["grid_length"] * marker_size[1]
marker_s = params["grid_length"] * marker_size[0]

marker_th_l_max = marker_l * 1.6
marker_th_s_max = marker_s * 1.6

marker_th_l_min = marker_l * 0.8
marker_th_s_min = marker_s * 0.8

# if the point clouds haven't been segmented, they will be processed
# not_segmented = params['not_segmented']
not_segmented = True
debug = False
# get vertical and horizontal scan resolution for jdc and agglomeration
if params['LiDAR_type'] == 'hdl32':
    h_coef = 2 * np.sin(np.deg2rad(360. / (70000. / 32.)) / 2)
    v_coef = 2 * np.sin(np.deg2rad(41.34 / 31.) / 2.)
elif params['LiDAR_type'] == 'hdl64':
    h_coef = 2 * np.sin(np.deg2rad(0.08) / 2)
    v_coef = 2 * np.sin(np.deg2rad(0.4 / 2.))
elif params['LiDAR_type'] == 'vlp16_puck':
    h_coef = 2 * np.sin(np.deg2rad(0.25) / 2)
    v_coef = 2 * np.sin(np.deg2rad(2) / 2.)
else:
    AssertionError("Please input the right LiDAR_type in the config.yaml")


# scanline segment class segmented scanline by scanline
class jdc_segment:
    def __init__(self, laser_id=-1, segment_id=-1, points_xyz=np.array([-1, -1, -1]), points_rgb=np.array([255, 0, 0])):
        self.laser_id = laser_id
        self.segment_id = segment_id
        self.points_xyz = points_xyz
        self.points_rgb = points_rgb
        self.is_potential = False
        self.centroid = np.array([0, 0, 0])

    def update_centroid(self):  # calc the centroid of a segment by calculating the average of points in seg
        self.centroid = self.points_xyz.mean(0)

    def calc_integrate_len(self):
        points_num = self.points_xyz.shape[0]
        total_length = 0
        for i in xrange(points_num - 1):
            total_length = total_length + LA.norm(self.points_xyz[i] - self.points_xyz[i + 1])
        return total_length

    def get_points_num(self):
        return self.points_xyz.shape[0]

    def calc_aver_angle_dif(self):
        points_num = self.points_xyz.shape[0]
        ang_dif_list = list()
        if points_num < 3:
            ang_dif_list = list([1, 1, 1])
            # print "points_num="+str(points_num)
        else:
            for i in np.arange(1, points_num - 1):
                a = (self.points_xyz[i] - self.points_xyz[i - 1]) * 100
                b = (self.points_xyz[i + 1] - self.points_xyz[i]) * 100
                ang_dif_cos = np.dot(a, b) / (LA.norm(a) * LA.norm(b))
                ang_dif_list.append(ang_dif_cos)
        if len(ang_dif_list) > 1:
            ang_dif_list.remove(min(ang_dif_list))
        return np.array(ang_dif_list).mean()


# segment class agglomerated from the scanline segments
class jdc_segments_collection:
    def __init__(self):
        self.segs_list = list()
        self.__plane_detection_points_thre = 30
        self.__normals_list = list()
        self.__jdc_thre_ratio = params['jdc_thre_ratio']
        self.__jdc_angle_thre = 0.5
        self.__csv_path = ""
        self.__palnar_normal_num = 100
        self.ransac_distance_threshold = 0.1
        self.__human_length_th_lower = 0
        self.__human_length_th_upper = 60
        self.__point_in_plane_threshold = 0.01
        self.__random_seg_color = True
        self.__agglomerative_cluster_th_ratio = params['agglomerative_cluster_th_ratio']
        self.l = params['laser_beams_num']
        self.__g_th = -1000
        self.__r_th = -1000
        self.__horizontal_scan_coef = h_coef
        self.__vertical_scan_coef = v_coef

    def set_random_color(self, t_f):
        self.__random_seg_color = t_f

    def add_seg(self, jdc_seg):
        self.segs_list.append(jdc_seg)

    def set_csv_path(self, csv_path):
        self.__csv_path = csv_path

    def is_point_in_plane(self, points_arr):
        planes_num = len(self.__normals_list)
        result = False
        for i in xrange(planes_num):
            plane_normal = self.__normals_list[i]
            error = abs(np.asmatrix(np.array(plane_normal)[:3]) * points_arr.T + plane_normal[3]).mean()
            # print error
            # print "test"
            if error < self.__point_in_plane_threshold:
                result = True
                break
        return result

    def exact_planar_normals(self):
        import pcl
        if self.__csv_path == "":
            print "csv file path is not spcified!"
        raw_data = np.genfromtxt(self.__csv_path, delimiter=",", skip_header=1)
        points_xyz_arr = np.array(raw_data[:, :3], dtype=np.float32)
        points_cloud = pcl.PointCloud()
        points_cloud.from_array(points_xyz_arr)
        for i in xrange(self.__palnar_normal_num):
            seg = points_cloud.make_segmenter()
            seg.set_optimize_coefficients(True)
            seg.set_model_type(pcl.SACMODEL_PLANE)
            seg.set_method_type(pcl.SAC_RANSAC)
            seg.set_distance_threshold(self.ransac_distance_threshold)
            indices, model = seg.segment()
            if len(indices) < self.__plane_detection_points_thre:
                break
            # model turns Hessian Normal Form of a plane in 3D
            # http://mathworld.wolfram.com/HessianNormalForm.html
            self.__normals_list.append(model)
            tmp = points_cloud.to_array()
            tmp = np.delete(tmp, indices, 0)
            points_cloud.from_array(tmp)
            # show_xyzrgb_points_vtk(tmp)

    def get_potential_segments(self):
        laser_id_col_ind = 4
        with open(self.__csv_path) as f:
            first_line = f.readline()
        header_itmes = first_line.split(",")
        for i in xrange(len(header_itmes)):
            if header_itmes[i].find("laser_id") > -1:
                print "laser_id is found in ", i, "-th colunm!"
                laser_id_col_ind = i
                break
        else:
            warnings.warn(
                "laser_id is not found in the hearder of the csv file. This may cause the point cloud failed to segment.",
                UserWarning)

        raw_data = np.genfromtxt(self.__csv_path, delimiter=",", skip_header=1)

        self.__r_th = max(raw_data[:, 2])
        self.__g_th = min(raw_data[:, 2])

        l = self.l

        for m in xrange(l):
            order = np.where(raw_data[:, laser_id_col_ind] == m)  # the 4-th column means the laser id
            temp_data = raw_data[order[0]][:, :3]  # exact XYZ data with laser_id=i
            data_arr_by_id_list = temp_data
            total_points_num = data_arr_by_id_list.shape[0]
            start_point_pos = 0
            stop_point_pos = 0
            first_found = False
            first_jdc = -1
            for i in xrange(total_points_num):
                #                 print i
                a = data_arr_by_id_list[i, :]
                if i < total_points_num - 1:
                    b = data_arr_by_id_list[i + 1, :]
                else:
                    b = data_arr_by_id_list[0, :]
                dis = LA.norm(a - b)
                current_thre = self.__horizontal_scan_coef * LA.norm(a) * self.__jdc_thre_ratio
                if dis >= current_thre:

                    stop_point_pos = i
                    jdc_seg = jdc_segment()
                    jdc_seg.laser_id = m
                    jdc_seg.segment_id = i
                    jdc_seg.points_xyz = data_arr_by_id_list[start_point_pos:stop_point_pos + 1, :]

                    jdc_seg.update_centroid()

                    jdc_seg.points_rgb = np.random.randint(0, 255, size=3)
                    least_points_num = 0

                    if len(jdc_seg.points_xyz) > least_points_num:
                        if not first_found:
                            first_jdc = i
                            first_found = True
                        else:
                            self.add_seg(jdc_seg)
                    del jdc_seg
                    start_point_pos = i + 1
                if i == total_points_num - 1:
                    jdc_seg = jdc_segment()
                    jdc_seg.laser_id = m
                    jdc_seg.segment_id = i
                    jdc_seg.points_xyz = np.vstack(
                        [data_arr_by_id_list[start_point_pos:i + 1, :], data_arr_by_id_list[:first_jdc + 1, :]])
                    if len(jdc_seg.points_xyz) > least_points_num:
                        self.add_seg(jdc_seg)
                    del jdc_seg

    def return_potential_seg(self):
        pot_seg = list()
        for i in xrange(len(self.segs_list)):
            tmp = self.segs_list[i]
            if tmp.is_potential:
                tmp.points_rgb = np.array([255, 0, 0])
            pot_seg.append(tmp)
        return pot_seg

    def cluster_seg(self):
        clustered_list = list()
        print "jdc number: ", len(self.segs_list)
        copy_segs_list = copy.deepcopy(self.segs_list)
        z_list = []
        for segs in copy_segs_list:
            z_list.append(segs.centroid[2])
        order_z = np.argsort(z_list)
        sorted_segs_list = []
        for ele in order_z:
            sorted_segs_list.append(copy_segs_list[ele])
        # g_list = list()
        # r_list = list()
        # for i in range(len(sorted_segs_list) - 1, -1, -1):
        #     if sorted_segs_list[i].centroid[2] <= self.__g_th + 0.01:
        #         g_list.append(sorted_segs_list[i])
        #         sorted_segs_list.pop(i)
        #     elif sorted_segs_list[i].centroid[2] >= self.__r_th - 0.01:
        #         r_list.append(sorted_segs_list[i])
        #         sorted_segs_list.pop(i)
        # print "glist", len(g_list), "r_list", len(r_list)
        # clustered_list.append(g_list)
        # clustered_list.append(r_list)
        #
        searchlist = np.arange(len(sorted_segs_list)).tolist()
        while len(searchlist) > 0:
            clustered_seg = list()
            search_num = len(searchlist) - 2
            this_seg = sorted_segs_list[searchlist[-1]]
            color_tup = np.array([np.random.randint(255), np.random.randint(255), np.random.randint(255)])
            this_seg.points_rgb = color_tup
            clustered_seg.append(this_seg)

            for i in np.arange(search_num, -1, -1):
                tmp_seg = sorted_segs_list[searchlist[i]]

                current_agglo_thre = self.__vertical_scan_coef * LA.norm(
                    tmp_seg.centroid) * self.__agglomerative_cluster_th_ratio
                # print "number of clusters in clustered_seg: "+str(len(clustered_seg))
                for each in clustered_seg:
                    if LA.norm(tmp_seg.centroid - each.centroid) < current_agglo_thre and calc_vectors_pca_correlation(
                            tmp_seg, each):
                        tmp_seg.points_rgb = color_tup
                        clustered_seg.append(tmp_seg)

                        searchlist.remove(searchlist[i])
                        break
            if len(clustered_seg) > 0:
                clustered_list.append(clustered_seg)
            searchlist.remove(searchlist[-1])
        print "seg_co was segmented into " + str(len(clustered_list))
        return clustered_list


# calcuate the least distance of points from two segments
def calc_min_len_of_seg_co(a, b):
    min_len = 1000000
    for i in a:
        for j in b:
            tmp = LA.norm(i - j)
            if tmp < min_len:
                min_len = tmp
    return min_len


# calcuate the correlation of segments according to their PCA decomposition
def calc_pca_correlation(a, b):
    a_list = list()
    b_list = list()
    for tmp in a:
        a_list.extend(tmp.points_xyz)
    for tmp in b:
        b_list.extend(tmp.points_xyz)
    a_arr = np.asarray(a_list)
    b_arr = np.asarray(b_list)

    sim_r_th = 0.5
    sim_b_th = 0.5
    a_arr = np.asarray(a_arr)
    b_arr = np.asarray(b_arr)
    #     print a_arr.shape
    #     print b_arr.shape
    if a_arr.shape[0] > 5 and b_arr.shape[0] > 5:
        pca_a = PCA(n_components=3)
        pca_a.fit(a_arr)
        pca_b = PCA(n_components=3)
        pca_b.fit(b_arr)
        #         print pca_a.components_
        #         print pca_b.components_
        sim_r = norm(pca_a.explained_variance_ratio_ - pca_b.explained_variance_ratio_)
        sim_b = 0
        for i in xrange(3):
            sim_b = sim_b + abs((pca_a.explained_variance_ratio_[i] + pca_b.explained_variance_ratio_[i]) / 2 * (
                np.dot(pca_a.components_[i], pca_b.components_[i])) / (
                                        norm(pca_a.components_[i]) * norm(pca_b.components_[i])))
        # print sim_b
        if sim_r < sim_r_th and sim_b > sim_b_th:
            return True
        else:
            return False
    else:
        return False


# calculate the correlation of tow scanline segments
def calc_vectors_pca_correlation(a, b):
    sim_r_th = 0.2  # 0.5
    sim_b_th = 0.9  # 0.5
    a_arr = np.asarray(a.points_xyz)
    b_arr = np.asarray(b.points_xyz)

    if a_arr.shape[0] > 5 and b_arr.shape[0] > 5:
        pca_a = PCA(n_components=3)
        pca_a.fit(a_arr)
        pca_b = PCA(n_components=3)
        pca_b.fit(b_arr)
        #         print pca_a.components_
        #         print pca_b.components_
        sim_r = norm(pca_a.explained_variance_ratio_ - pca_b.explained_variance_ratio_)
        sim_b = 0
        for i in xrange(3):
            sim_b = sim_b + abs((pca_a.explained_variance_ratio_[i] + pca_b.explained_variance_ratio_[i]) / 2 * (
                np.dot(pca_a.components_[i], pca_b.components_[i])) / (
                                        np.linalg.norm(pca_a.components_[i]) * np.linalg.norm(pca_b.components_[i])))
        if sim_r < sim_r_th and sim_b > sim_b_th:
            return True
        else:
            return False
    else:
        return False


if debug:
    import vtk


    def show_pcd_ndarray(array_data, color_arr=[0, 255, 0]):
        all_rows = array_data.shape[0]
        Colors = vtk.vtkUnsignedCharArray()
        Colors.SetNumberOfComponents(3)
        Colors.SetName("Colors")

        Points = vtk.vtkPoints()
        Vertices = vtk.vtkCellArray()

        for k in xrange(all_rows):
            point = array_data[k, :]
            id = Points.InsertNextPoint(point[0], point[1], point[2])
            Vertices.InsertNextCell(1)
            Vertices.InsertCellPoint(id)
            if vtk.VTK_MAJOR_VERSION > 6:
                Colors.InsertNextTuple(color_arr)
            else:
                Colors.InsertNextTupleValue(color_arr)

            dis_tmp = np.sqrt((point ** 2).sum(0))
            # Colors.InsertNextTupleValue([0,255-dis_tmp/max_dist*255,0])
            # Colors.InsertNextTupleValue([255-abs(point[0]/x_max*255),255-abs(point[1]/y_max*255),255-abs(point[2]/z_max*255)])
            # Colors.InsertNextTupleValue([255-abs(point[0]/x_max*255),255,255])

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(Points)
        polydata.SetVerts(Vertices)
        polydata.GetPointData().SetScalars(Colors)
        polydata.Modified()

        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(polydata)
        else:
            mapper.SetInputData(polydata)
        mapper.SetColorModeToDefault()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(5)

        # Renderer
        renderer = vtk.vtkRenderer()
        renderer.AddActor(actor)
        renderer.SetBackground(.2, .3, .4)
        renderer.ResetCamera()

        # Render Window
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)

        # Interactor
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)

        # Begin Interaction
        renderWindow.Render()
        renderWindowInteractor.Start()


# determine whether a segment is the potential chessboard's point cloud
def is_marker(file_full_path, range_res, points_num_th=250):
    # result = False
    jdcs_collection = cPickle.load(open(file_full_path, 'rb'))

    if debug:
        print file_full_path
    tmp_list = list()
    for jdc in jdcs_collection:
        tmp_list.extend(jdc)
    arr = np.array(tmp_list)
    if debug:
        show_pcd_ndarray(arr)
    if arr.shape[0] < points_num_th:
        if debug:
            print "points num: ", arr.shape[0]
        return False

    # use the distance between the marker's center and the lidar to filter
    avg = arr.mean(axis=0)
    if np.linalg.norm(avg) > range_res:
        if debug:
            print "avg: ", np.linalg.norm(avg)
        return False

    # check whether is a plane
    pca = PCA(n_components=3)
    pca.fit(arr)
    if pca.explained_variance_ratio_[2] > params['chessboard_detect_planar_PCA_ratio']:
        if debug:
            print "pca: ", pca.explained_variance_ratio_
        return False

    # map to 2D
    tmp = np.dot(pca.components_, arr.T).T
    points = tmp[:, :2]

    # substract the mean point
    points -= points.mean(axis=0)

    bbx = points.max(axis=0) - points.min(axis=0)
    if debug:
        print "bbx: ", bbx

    if (marker_th_l_min < bbx[0] < marker_th_l_max and marker_th_s_min < bbx[1] < marker_th_s_max) or (
            marker_th_s_min < bbx[0] < marker_th_s_max and marker_th_l_min < bbx[1] < marker_th_l_max):
        # analyse the distribution of the points in four quadrants
        x_lin = [points.min(axis=0)[0], (points.min(axis=0)[0] + points.max(axis=0)[0]) / 2, points.max(axis=0)[0]]
        y_lin = [points.min(axis=0)[1], (points.min(axis=0)[1] + points.max(axis=0)[1]) / 2, points.max(axis=0)[1]]

        num_in_quadrant_ls = []
        for i in xrange(2):
            x_prd = [x_lin[i], x_lin[i + 1]]
            for j in xrange(2):
                y_prd = [y_lin[j], y_lin[j + 1]]
                num_in_quadrant_ls.append(np.count_nonzero(
                    (points[:, 0] >= x_prd[0]) & (points[:, 0] <= x_prd[1]) & (points[:, 1] >= y_prd[0]) & (
                            points[:, 1] <= y_prd[1])))
        normed = np.array(num_in_quadrant_ls, dtype=np.float32) / sum(num_in_quadrant_ls)

        if normed.max() - normed.min() < 0.15:
            print file_full_path
            print "passed"
            print "pca: ", pca.explained_variance_ratio_
            if debug:
                show_pcd_ndarray(arr)
            return True
        else:
            return False
    else:
        # print "over length of diagonal line"
        return False


# find the point cloud of chessboard from segmented results
def find_marker(file_path, csv_path, range_res=params['marker_range_limit']):
    file_list = os.listdir(file_path)
    res_ls = []
    for file in file_list:
        # print  file_path + file
        if is_marker(file_path + file, range_res):
            # print file
            res_ls.append(file_path + file)
    print len(res_ls)
    if len(res_ls) == 0:
        AssertionError("no marker is found")
    if len(res_ls) > 1:
        print "one than one candicate of the marker is found!"
        print res_ls
        print "The segment with most uniform intensity distribution is considered as the marker"
        num_ls = []
        for file in res_ls:
            arr = exact_full_marker_data(csv_path, [file])
            intensity_arr = arr[:, 3]
            hist, bin_edges = np.histogram(intensity_arr, 100)
            if debug:
                print hist, bin_edges
            num_ls.append(len(np.nonzero(hist)[0]))
        res_ls = [res_ls[np.argmax(num_ls)]]
        if debug:
            print res_ls

    assert len(res_ls) == 1
    print "marker is found!"
    return res_ls


# get the reflectance information of the chessboard's point cloud
def exact_full_marker_data(csv_path, marker_pkl):
    all_data_arr = np.genfromtxt(csv_path, delimiter=",", skip_header=1)
    marker_jdcs_collection = cPickle.load(open(marker_pkl[0], "rb"))
    tmp_list = list()
    for jdc in marker_jdcs_collection:
        tmp_list.extend(jdc)
    marker_pcd_arr = np.array(tmp_list)

    tree = spatial.KDTree(all_data_arr[:, :3])
    marker_full_data_ls = []
    for i in xrange(marker_pcd_arr.shape[0]):
        ret = tree.query(marker_pcd_arr[i])
        marker_full_data_ls.append(all_data_arr[ret[1]])

    marker_full_data_arr = np.array(marker_full_data_ls)
    return marker_full_data_arr


# get the Hessian normal form of 3D plane
def get_plane_model(arr):
    import pcl
    ransac_distance_threshold = 0.05
    point_cloud = pcl.PointCloud(arr.astype(np.float32))
    seg = point_cloud.make_segmenter_normals(ksearch=50)

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(10000)
    seg.set_distance_threshold(ransac_distance_threshold)
    indices, model = seg.segment()
    print "percentage of points in plane model: ", np.float32(len(indices)) / arr.shape[0]
    return model


# project a point to an estimated plane
def p2pl_proj(pl_norm, pl_p, p):
    v = np.array(p) - np.array(pl_p)
    dis = np.dot(v, pl_norm)
    res = p - dis * pl_norm
    return res


# estimate the mean of the low and high intensity of the chessboard's points
def get_gmm_para(arr):
    from sklearn import mixture
    gmm = mixture.GaussianMixture(n_components=2, covariance_type="diag", max_iter=10000,
                                  means_init=np.array([[5], [60]])).fit(arr)
    return gmm


# transfter the points of the chessboard to chessboard plane
# for implementation convenience, the vector with the largest ratio is mapped to the y-axis and the vector with the
# second ratio is mapped to x-axis, which is different from the explaination in the paper
def transfer_by_pca(arr):
    # to roate the arr correctly, the direction of the axis has to be found correct(PCA also shows the aixs but not the direction of axis)
    #
    pca = PCA(n_components=3)
    pca.fit(arr)

    ####################################################
    # there are there requirements for the coordinates axes for the coordinate system of the chessboard
    # 1. right-hand rule
    # 2. z axis should point to the origin
    # 3. the angle between y axis of chessboard and z axis of velodyne less than 90 deg
    ####################################################
    trans_mat = pca.components_
    # swith x and y axis
    trans_mat[[0, 1]] = trans_mat[[1, 0]]
    # cal z axis to obey the right hands
    trans_mat[2] = np.cross(trans_mat[0], trans_mat[1])

    # to make angle between y axis of chessboard and z axis of velodyne less than 90 deg
    sign2 = np.sign(np.dot(trans_mat[1], np.array([0, 0, 1])))
    # print "sign2", sign2
    trans_mat[[0, 1]] = sign2 * trans_mat[[0, 1]]

    # to make the norm vector point to the side where the origin exists
    # the angle  between z axis and the vector  from one point on the board to the origin should  be less than 90 deg
    sign = np.sign(np.dot(trans_mat[2], 0 - arr.mean(axis=0).T))

    # print "sign", sign
    trans_mat[[0, 2]] = sign * trans_mat[[0, 2]]

    tmp = np.dot(arr, trans_mat.T)
    # print pca.components_
    # print "x,y,cross", np.cross(pca.components_[1], pca.components_[2])

    return trans_mat, tmp


# cost function for fitting the chessboard model and the chessboard's point cloud
def cost_func_for_opt_mini(theta_t, transed_pcd, marker_full_data_arr, gray_zone, x_res=marker_size[0],
                           y_res=marker_size[1],
                           grid_len=params["grid_length"]):  # ls: grids coords(not used), arr: markers arr

    transed_pcd_for_costf = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], theta_t[0]),
                                   (transed_pcd + np.array([[theta_t[1], theta_t[2], 0]])).T).T
    arr = np.hstack([transed_pcd_for_costf, marker_full_data_arr[:, 3:]])
    bound = np.array([[0, 0], [0, y_res], [x_res, y_res], [x_res, 0]]) * grid_len - np.array([x_res,
                                                                                              y_res]) * grid_len / 2

    x_grid_arr = (np.array(range(0, x_res + 1)) - float(x_res) / 2) * grid_len
    y_grid_arr = (np.array(range(0, y_res + 1)) - float(y_res) / 2) * grid_len

    x = range(arr.shape[0])
    y = []
    polygon_path = mplPath.Path(bound)
    cost = 0
    for row in arr:
        if polygon_path.contains_point(row[:2]):
            if gray_zone[0] < row[params['intensity_col_ind']] < gray_zone[1]:
                y.append(0.5)
                continue
            else:
                i = int((row[0] + x_res * grid_len / 2) / grid_len)
                j = int((row[1] + y_res * grid_len / 2) / grid_len)
                if i % 2 == 0:
                    if j % 2 == 0:
                        color = 0
                    else:
                        color = 1
                else:
                    if j % 2 == 0:
                        color = 1
                    else:
                        color = 0

                estimated_color = (np.sign(row[params['intensity_col_ind']] - gray_zone[1]) + 1) / 2
                if estimated_color != color:
                    cost += (min(abs(row[0] - x_grid_arr)) + min(abs(row[1] - y_grid_arr)))
                y.append(color)
        else:
            cost += (min(abs(row[0] - x_grid_arr)) + min(abs(row[1] - y_grid_arr)))

            y.append(2)

    return cost


# create the chessboard model
def generate_grid_coords(x_res=marker_size[0], y_res=marker_size[1], grid_len=params['grid_length']):  # res, resolution

    ls = []
    for i in xrange(x_res):
        for j in xrange(y_res):
            orig = np.array([i, j, 0]) * grid_len - np.array([x_res, y_res, 0]) * grid_len / 2
            p1 = np.array([i + 1, j, 0]) * grid_len - np.array([x_res, y_res, 0]) * grid_len / 2
            p2 = np.array([i, j + 1, 0]) * grid_len - np.array([x_res, y_res, 0]) * grid_len / 2
            if i % 2 == params['start_pattern_corner']:
                if j % 2 == params['start_pattern_corner']:
                    color = params['start_pattern_corner']
                else:
                    color = 1 - params['start_pattern_corner']
            else:
                if j % 2 == params['start_pattern_corner']:
                    color = 1 - params['start_pattern_corner']
                else:
                    color = params['start_pattern_corner']
            ls.append([orig, p1, p2, color])
    return ls


# analyze the intensity distribution of the chessboard's point cloud to determine the gray zone
def get_gray_thre(intes_arr):
    gray_zone_debug = False
    # find the gray period of intensity (if some point has the intensity in this period, the weight for this kind of pints will be zero)
    gmm = get_gmm_para(np.expand_dims(intes_arr, axis=1))
    tmp_thres = gmm.means_.mean()
    if gray_zone_debug:
        print "Mean of intensity by GMM: ", tmp_thres

    hist, bin_edges = np.histogram(intes_arr, 100)
    if gray_zone_debug:
        import matplotlib.pyplot as plt
        plt.hist(intes_arr, 100)
        plt.show()
    order = np.argsort(hist)

    low_intensity = -1
    high_intensity = -1

    low_found = False
    high_found = False
    for i in range(order.shape[0] - 1, -1, -1):
        if bin_edges[order[i]] > tmp_thres and not high_found:
            high_found = True
            high_intensity = bin_edges[order[i]]

        if bin_edges[order[i]] < tmp_thres and not low_found:
            low_found = True
            low_intensity = bin_edges[order[i]]

        if high_found and low_found:
            break
    else:
        print "gray zone is not well detected!"
        print low_intensity, high_intensity

    return low_intensity, high_intensity


# segment single frame of the point cloud into several segments
def seg_pcd(csv_path, save_folder_path):
    import pcl
    seg_num_thre = 3
    jdc_points_num_thres = 0
    seg_count = 0
    jdc_collection = jdc_segments_collection()
    jdc_collection.set_csv_path(csv_path)
    print "csv_file loaded!"
    jdc_collection.get_potential_segments()

    clustered_seg_list = jdc_collection.cluster_seg()
    potential_seg_co_list = list()
    for tmp_seg_co in clustered_seg_list:
        # if is_human(tmp_seg_co):
        # color_tuple=np.array([0,255,0])
        potential_seg_co_list.append(tmp_seg_co)
    twice_clustered_seg_list = clustered_seg_list

    print "twice_clustered_seg num=" + str(len(twice_clustered_seg_list))

    parts = csv_path.split("/")
    if os.path.isdir(save_folder_path + parts[-1].split(".")[0]):
        shutil.rmtree(save_folder_path + parts[-1].split(".")[0])
    os.makedirs(save_folder_path + parts[-1].split(".")[0])

    count_after_filter = 0
    for tmp_seg_co in twice_clustered_seg_list:
        if len(tmp_seg_co) > seg_num_thre:
            count_after_filter += 1
            list_for_pedestrians_pcd = list()
            list_for_jdcs = list()
            for j in xrange(len(tmp_seg_co)):
                tmp_seg = tmp_seg_co[j]
                list_for_jdcs.append(tmp_seg.points_xyz.tolist())
                for k in xrange(tmp_seg.points_xyz.shape[0]):
                    point = tmp_seg.points_xyz[k, :]
                    list_for_pedestrians_pcd.append(point)
            arr_for_pedestrians_pcd = np.asarray(list_for_pedestrians_pcd, dtype=np.float32)
            if arr_for_pedestrians_pcd.shape[0] > 0:
                pcd_pedestrian = pcl.PointCloud(arr_for_pedestrians_pcd)

                parts = csv_path.split("/")

                if pcd_pedestrian.size > jdc_points_num_thres:
                    save_path_for_pedestrian_txt = save_folder_path + "/" + parts[-1].split(".")[0] + "/" + \
                                                   parts[-1].split(".")[0] + "block" + str(
                        seg_count) + ".txt"
                    seg_count += 1
                    cPickle.dump(list_for_jdcs, open(save_path_for_pedestrian_txt, 'wb'))
            del arr_for_pedestrians_pcd
            del list_for_pedestrians_pcd
            del list_for_jdcs
    del jdc_collection


# optimize to find the pose solution that makes the chessboard model fit the detected chessboard's point cloud best
def opt_min(param_ls, initial_guess=np.zeros(3).tolist()):
    method = param_ls[0]
    try:
        res = minimize(cost_func_for_opt_mini, initial_guess, args=param_ls[1],
                       method=method, tol=1e-10, options={"maxiter": 10000000})  # , "disp": True

        print method, ": ", res.fun, "  ", res.x
        return res.fun, [method, res]
    except:
        print method, ": could not be applied"
        return None


# utilize the defined functions to get the chessboard's corners for single frame point cloud
def run(csv_path, save_folder_path=os.path.join(params['base_dir'], "output/pcd_seg/"), size=marker_size):
    if not_segmented:
        seg_pcd(csv_path, save_folder_path)
    parts = csv_path.split("/")
    find_marker_path = save_folder_path + parts[-1].split(".")[0] + "/"

    marker_pkl = find_marker(file_path=os.path.abspath(find_marker_path) + "/", csv_path=csv_path)
    marker_full_data_arr = exact_full_marker_data(csv_path, marker_pkl)

    # fit the points to the plane model
    model = get_plane_model(marker_full_data_arr[:, :3])
    pl_p = np.array([0, 0, -model[3] / model[2]])  # a point on the plane of the model
    normal = np.array(model[:3])
    fitted_list = []
    for i in marker_full_data_arr[:, :3]:
        b = p2pl_proj(normal, pl_p, i)
        fitted_list.append(b)
    marker_data_arr_fitted = np.array(fitted_list)
    marker_full_data_arr_fitted = np.hstack([marker_data_arr_fitted, marker_full_data_arr[:, 3:]])

    # trans chessboard
    if 1:
        # render for model of checkerboard
        rot1, transed_pcd = transfer_by_pca(marker_data_arr_fitted)
        t1 = transed_pcd.mean(axis=0)
        transed_pcd = transed_pcd - t1

    # calculate the rotate angle in xoy palne around the z axis
    if 1:
        low_intes, high_intens = get_gray_thre(marker_full_data_arr_fitted[:, params['intensity_col_ind']])
        print "low_intes,high_intes:", low_intes, high_intens
        rate = 2
        gray_zone = np.array([((rate - 1) * low_intes + high_intens), (low_intes + (rate - 1) * high_intens)]) / rate

        methods = ['Powell']
        res_dict = {}

        # for parallel processing
        args = (transed_pcd, marker_full_data_arr, gray_zone,)
        param_ls = [[method, args] for method in methods]
        res_ls = map(opt_min, param_ls)
        for item in res_ls:
            if item is not None:
                res_dict[item[0]] = item[1]

        res = res_dict[min(res_dict)][1]

        print res_dict[min(res_dict)][0]
        print res

        rot2 = transforms3d.axangles.axangle2mat([0, 0, 1], res.x[0])
        t2 = np.array([res.x[1], res.x[2], 0])

        if 1:
            transed_pcd = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], res.x[0]),
                                 (transed_pcd + np.array([[res.x[1], res.x[2], 0]])).T).T

        gird_coords = generate_grid_coords()
        grid_ls = [(p[0]).flatten()[:2] for p in gird_coords]
        corner_arr = np.transpose(np.array(grid_ls).reshape(size[0], size[1], 2)[1:, 1:], (1, 0, 2))

    return [rot1, t1, rot2, t2, corner_arr, res.x, os.path.relpath(marker_pkl[0])]


# for multiple  processing
def main_for_pool(i):
    pcd_file = os.path.join(params['base_dir'], "pcd/") + str(i).zfill(params["file_name_digits"]) + ".csv"
    print pcd_file
    try:
        result = run(csv_path=pcd_file)
        print result
        save_file_path = os.path.join(params['base_dir'], "output/pcd_seg/") + str(i).zfill(
            params["file_name_digits"]) + "_pcd_result.pkl"
        with open(os.path.abspath(save_file_path), 'w') as file:
            file.truncate()
            cPickle.dump(result, file)
        print "pkl file was saved to " + save_file_path + " successfully!"
        print
        print
    except AssertionError:
        print "marker cannot be found"
        print "skip " + pcd_file


# main function for detecting corners from pcd files in the folder
def detect_pcd_corners():
    file_ls = os.listdir(os.path.join(params['base_dir'], "pcd"))
    pcd_ls = []
    for file in file_ls:
        if file.find("csv") > -1:
            pcd_ls.append(int(re.findall(r'\d+', file)[0]))
    if params["multi_proc"]:
        pool = Pool(params["proc_num"])
        pool.map(main_for_pool, pcd_ls)
    else:
        for ind in pcd_ls:
            main_for_pool(ind)


if __name__ == "__main__":
    detect_pcd_corners()
    # main_for_pool(1)
