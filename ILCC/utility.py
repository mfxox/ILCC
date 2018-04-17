# coding=utf-8
'''
Created on 3/20/2017 8:58 57PM Wang Weimin

@author: wangwm
'''
import os
from pcd_corners_est import exact_full_marker_data
import numpy as np
from pcd_corners_est import generate_grid_coords
import matplotlib.pyplot as plt
import matplotlib
import vtk
import config
from ast import literal_eval as make_tuple
import cPickle
import cv2
from LM_opt import xyz2angle, voxel2pixel
import transforms3d
from matplotlib.pyplot import cm
import ast

from sklearn.decomposition import PCA
import matplotlib.path as mplPath
import warnings

params = config.default_params()
marker_size = make_tuple(params["pattern_size"])
(H, W) = make_tuple(params['image_res'])

matplotlib.rcParams['text.usetex'] = True
matplotlib.rcParams['text.latex.unicode'] = True
plt.style.use("ggplot")

axis_font = {'fontname': 'Arial', 'size': '35'}


def draw_one_grd_vtk(ls):  # arr:[a,b,c,d],a:orig, b, point1, c,point 2, d,color
    source = vtk.vtkPlaneSource()
    source.SetOrigin(ls[0])
    source.SetPoint1(ls[1])
    source.SetPoint2(ls[2])
    source.Update()
    # source.SetPoint1(0, 0, 0)
    # source.SetPoint2(4, 3, 0)

    # mapper
    mapper = vtk.vtkPolyDataMapper()
    color = vtk.vtkUnsignedCharArray()
    color.SetName("colors")
    color.SetNumberOfComponents(3)

    # color_tup = np.random.randint(1, 255, 3)

    color.SetNumberOfTuples(source.GetOutput().GetNumberOfCells())
    for i in xrange(source.GetOutput().GetNumberOfCells()):
        color_tup = np.array([255, 255, 255]) * ls[3]
        color.InsertTuple(i, color_tup)

    source.GetOutput().GetCellData().SetScalars(color)

    mapper.SetInputConnection(source.GetOutputPort())

    # actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # assign actor to the renderer
    # ren.AddActor(actor)

    return actor


# generate the color list of the point cloud for different color styles. intens_rg: color by reflectance intensity (red:high green:low),
# intens: color by reflectance intensity (white:high back:low), autumn: matplotlib autumn color map,  cool: matplotlib cool color map
def gen_color_tup_for_vis(color_style="intens_rg", xyzi_arr=None):
    assert xyzi_arr is not None, "The array of the point cloud must be not None"
    a = xyzi_arr[:, 3].min()
    b = xyzi_arr[:, 3].max()
    color_ls = []
    if color_style == "intens_rg":
        tmp = (xyzi_arr[:, 3] - a) / (b - a) * 255
        for k in xrange(xyzi_arr.shape[0]):
            rgb_tuple = np.array([tmp[k], 0, 255 - xyzi_arr[k, 3]]).astype(np.int32)
            color_ls.append(rgb_tuple)
        return color_ls
    elif color_style == "intens":
        tmp = (xyzi_arr[:, 3] - a) / (b - a) * 255
        for k in xrange(xyzi_arr.shape[0]):
            rgb_tuple = np.repeat(tmp[k], 3).astype(np.int32)
            color_ls.append(rgb_tuple)
        return color_ls
    elif color_style == "autumn":
        tmp = (xyzi_arr[:, 3] - a).astype(np.float32) / (b - a)
        for k in xrange(xyzi_arr.shape[0]):
            rgb_tuple = (np.array(plt.cm.autumn(1 - tmp[k]))[:3] * 255).astype(np.int32)
            color_ls.append(rgb_tuple)
        return color_ls
    elif color_style == "cool":
        tmp = (xyzi_arr[:, 3] - a).astype(np.float32) / (b - a)
        for k in xrange(xyzi_arr.shape[0]):
            rgb_tuple = (np.array(plt.cm.cool(tmp[k]))[:3] * 255).astype(np.int32)
            color_ls.append(rgb_tuple)
        return color_ls
    elif color_style == "monochrome":
        # color = (np.random.randint(0, 255, 3)).tolist()
        color = [46, 204, 113]
        for k in xrange(xyzi_arr.shape[0]):
            color_ls.append(color)
        return color_ls
    elif color_style == "by_height":
        low_height = xyzi_arr[:, 2].min()
        high_height = xyzi_arr[:, 2].max()
        tmp = 0.0 + 0.7 * (xyzi_arr[:, 2] - low_height) / (high_height - low_height)
        for k in xrange(xyzi_arr.shape[0]):
            rgb_tuple = (np.array(plt.cm.hsv(tmp[k]))[:3] * 255).astype(np.int32)
            color_ls.append(rgb_tuple)
        return color_ls
    else:
        raise ValueError('Input color type is not correct!')


# visualize 3D points with specified color style
def vis_3D_points(full_lidar_arr, color_style="intens_rg"):
    all_rows = full_lidar_arr.shape[0]
    Colors = vtk.vtkUnsignedCharArray()
    Colors.SetNumberOfComponents(3)
    Colors.SetName("Colors")
    Points = vtk.vtkPoints()
    Vertices = vtk.vtkCellArray()

    tuple_ls = gen_color_tup_for_vis(color_style, xyzi_arr=full_lidar_arr)

    for k in xrange(all_rows):
        point = full_lidar_arr[k, :3]
        id = Points.InsertNextPoint(point[0], point[1], point[2])
        Vertices.InsertNextCell(1)
        Vertices.InsertCellPoint(id)

        rgb_tuple = tuple_ls[k]
        if vtk.VTK_MAJOR_VERSION >= 7:
            Colors.InsertNextTuple(rgb_tuple)
        else:
            Colors.InsertNextTupleValue(rgb_tuple)
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(Points)
    polydata.SetVerts(Vertices)
    polydata.GetPointData().SetScalars(Colors)
    polydata.Modified()

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION < 6:
        mapper.SetInput(polydata)
    else:
        mapper.SetInputData(polydata)
    mapper.SetColorModeToDefault()
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetPointSize(8)

    return actor


# visualize 3D points with specified color array
def vis_pcd_color_arr(array_data, color_arr=[46, 204, 113]):
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
        if vtk.VTK_MAJOR_VERSION >= 7:
            Colors.InsertNextTuple(color_arr)
        else:
            Colors.InsertNextTupleValue(color_arr)
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
    actor.GetProperty().SetPointSize(10)
    return actor


# visualize with actor:
def vis_with_renderer(renderer):
    # Renderer

    # renderer.SetBackground(.2, .3, .4)
    renderer.SetBackground(1, 1, 1)
    renderer.ResetCamera()

    transform = vtk.vtkTransform()
    transform.Translate(1.0, 0.0, 0.0)
    axes = vtk.vtkAxesActor()
    renderer.AddActor(axes)

    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    def get_camera_info(obj, ev):
        if renderWindowInteractor.GetKeyCode() == "s":
            w2if = vtk.vtkWindowToImageFilter()
            w2if.SetInput(renderWindow)
            w2if.Update()

            writer = vtk.vtkPNGWriter()
            writer.SetFileName("screenshot.png")
            if vtk.VTK_MAJOR_VERSION == 5:
                writer.SetInput(w2if.GetOutput())
            else:
                writer.SetInputData(w2if.GetOutput())
            writer.Write()
            print "screenshot saved"

    style = vtk.vtkInteractorStyleSwitch()
    renderWindowInteractor.SetInteractorStyle(style)
    # style.SetCurrentStyleToTrackballActor()
    style.SetCurrentStyleToTrackballCamera()

    # Begin Interaction
    renderWindowInteractor.AddObserver(vtk.vtkCommand.KeyPressEvent, get_camera_info, 1)
    renderWindow.Render()
    renderWindowInteractor.Start()


def proj_pcd_2_pix(pcd_arr):
    if params["camera_type"] == "panoramic":
        angs_ls = map(xyz2angle, pcd_arr.tolist())
        pix_ls = (np.array(map(voxel2pixel, angs_ls))).tolist()
    elif params['camera_type'] == "perspective":
        intrinsic_paras_tuple = make_tuple(params['instrinsic_para'])
        intrinsic_paras = np.array(intrinsic_paras_tuple).reshape(3, 3)
        cam_coord_pcd = pcd_arr.copy()

        pcd_to_pix = (np.dot(intrinsic_paras, cam_coord_pcd.T)).T

        proj_pts = (pcd_to_pix / pcd_to_pix[:, 2].reshape(-1, 1))[:, :2].astype(np.int16)
        pix_ls = proj_pts.tolist()
    else:
        raise Exception("Camera type not correctly defined!")
    return pix_ls


def remove_occlusion_of_chessboard(pcd_arr, corners_in_pcd_arr):
    occlu_thres = 0.1
    pcd_ls = pcd_arr.tolist()
    pix_ls = proj_pcd_2_pix(pcd_arr)
    ind_ls = []
    pca = PCA(n_components=3)
    pca.fit(corners_in_pcd_arr)
    transed_corners_in_pcd_arr = np.dot(pca.components_, corners_in_pcd_arr.T).T
    center = transed_corners_in_pcd_arr.mean(axis=0)

    bound = np.dot(pca.components_.T,
                   (np.array(
                       [[-0.3, -0.225, 0], [-0.3, 0.225, 0], [0.3, 0.225, 0], [0.3, -0.225, 0]]) * 1.05 + center).T).T

    if params["camera_type"] == "panoramic":
        bound_on_image = np.fliplr(np.array(map(voxel2pixel, map(xyz2angle, bound.tolist()))))
    elif params['camera_type'] == "perspective":
        intrinsic_paras_tuple = make_tuple(params['instrinsic_para'])
        intrinsic_paras = np.array(intrinsic_paras_tuple).reshape(3, 3)

        pcd_to_pix = (np.dot(intrinsic_paras, bound.T)).T
        inds = np.where(pcd_arr[:, 2] > 0)
        pcd_ls = pcd_arr[inds].tolist()
        pix_ls = np.array(pix_ls)[inds].tolist()
        print "before removal: ", len(pcd_ls)
        proj_pts = (pcd_to_pix / pcd_to_pix[:, 2].reshape(-1, 1))[:, :2].astype(np.int16)
        bound_on_image = np.fliplr(proj_pts)
        # bound_on_image = proj_pts

        # print bound_on_image
    else:
        raise Exception("Camera type not correctly defined!")

    polygon_path = mplPath.Path(bound_on_image.tolist())

    for i in xrange(len(pcd_ls)):
        pix = list(reversed(pix_ls[i]))
        # print pix
        if polygon_path.contains_point(pix):
            point_2_board_dis = abs(np.dot(pca.components_[2], pcd_ls[i] - corners_in_pcd_arr.mean(axis=0)))
            # print point_2_board_dis
            # print pix_ls[i]
            if point_2_board_dis <= occlu_thres:
                if params["camera_type"] == "panoramic":
                    ind_ls.append(i)
                elif params['camera_type'] == "perspective":
                    ind_ls.append(inds[0][i])
                else:
                    raise Exception("Camera type not correctly defined!")
        else:
            if params["camera_type"] == "panoramic":
                ind_ls.append(i)
            elif params['camera_type'] == "perspective":
                ind_ls.append(inds[0][i])
            else:
                raise Exception("Camera type not correctly defined!")

    return np.array(ind_ls)


# visualize csv file of i-th point cloud
def vis_csv_pcd(ind=1, color_style="monochrome"):
    pcd_arr = np.genfromtxt("pcd/" + str(ind).zfill(4) + ".csv", delimiter=",", skip_header=1)
    # actor = vis_3D_points(pcd_arr, color_style="intens")
    actor = vis_3D_points(pcd_arr, color_style=color_style)
    renderer = vtk.vtkRenderer()
    renderer.AddActor(actor)
    vis_with_renderer(renderer)


def vis_segments(ind=1):
    renderer = vtk.vtkRenderer()
    seg_folder = "output/pcd_seg/" + str(ind).zfill(4) + "/"
    seg_list = os.listdir(seg_folder)
    for seg in seg_list:
        if seg.split(".")[-1] == "txt":
            color_tup = (np.random.randint(1, 255, 3)).tolist()
            points_ls = list()
            jdcs_collection = cPickle.load(open(os.path.abspath(seg_folder + seg), 'rb'))
            if len(jdcs_collection) > 0:  # filter
                for jdc in jdcs_collection:
                    points_ls.extend(jdc)
            # print points_ls
            actor = vis_pcd_color_arr(np.array(points_ls), color_tup)
            renderer.AddActor(actor)
    vis_with_renderer(renderer)


def vis_segments_only_chessboard_color(ind=1):
    renderer = vtk.vtkRenderer()
    seg_folder = "output/pcd_seg/" + str(ind).zfill(4) + "/"
    seg_list = os.listdir(seg_folder)
    chessboard_file_name = \
        cPickle.load(open("output/pcd_seg/" + str(ind).zfill(4) + "_pcd_result.pkl", "r"))[-1].split("/")[-1]
    for seg in seg_list:
        if seg.split(".")[-1] == "txt":
            if seg == chessboard_file_name:
                color_tup = np.array([0, 255, 0])
            else:
                color_tup = np.array([0, 0, 0])

            points_ls = list()
            jdcs_collection = cPickle.load(open(os.path.abspath(seg_folder + seg), 'rb'))
            if len(jdcs_collection) > 0:  # filter
                for jdc in jdcs_collection:
                    points_ls.extend(jdc)
            # print points_ls
            actor = vis_pcd_color_arr(np.array(points_ls), color_tup)
            renderer.AddActor(actor)
    vis_with_renderer(renderer)


def cal_theorical_number_points(dis):
    h_res = np.deg2rad(0.16)  # rad
    v_res = np.deg2rad(1.33)  # rad
    h_len = dis * h_res
    v_len = 2 * dis * np.sin(v_res / 2)
    w = 0.45
    l = 0.6
    return (l // v_len) * (w // h_len)


def vis_all_markers(ls=[1]):
    import vtk
    ren = vtk.vtkRenderer()
    # ren.SetBackground(.2, .3, .4)
    ren.SetBackground(.5, .6, .7)

    for i in ls:
        pcd_result_file = "output/pcd_seg/" + str(i).zfill(4) + "_pcd_result.pkl"
        csv_path = "pcd/" + str(i).zfill(4) + ".csv"

        with open(os.path.abspath(pcd_result_file), "r") as f:
            pcd_result_ls = cPickle.load(f)
        assert pcd_result_ls is not None

        marker_full_data_arr = exact_full_marker_data(csv_path, [pcd_result_ls[-1]])

        marker_arr = marker_full_data_arr[:, :3]
        # transformed_pcd = roate_with_rt(np.array(r_t), marker_arr)
        if i % 4 == 0:
            actor2 = vis_3D_points(
                np.hstack([marker_arr + np.array([0, 0, 0]), marker_full_data_arr[:, 3:]]), color_style="intens")
        elif i % 4 == 1:
            actor2 = vis_3D_points(
                np.hstack([marker_arr + np.array([0, 0, 0]), marker_full_data_arr[:, 3:]]), color_style="autumn")
        elif i % 4 == 2:
            actor2 = vis_3D_points(
                np.hstack([marker_arr + np.array([0, 0, 0]), marker_full_data_arr[:, 3:]]), color_style="cool")
        else:
            actor2 = vis_3D_points(
                np.hstack([marker_arr + np.array([0, 0, 0]), marker_full_data_arr[:, 3:]]),
                color_style="intens_rg")
        ren.AddActor(actor2)
    transform2 = vtk.vtkTransform()
    transform2.Translate(0.0, 0.0, 0.0)
    axes2 = vtk.vtkAxesActor()
    axes2.SetUserTransform(transform2)
    ren.AddActor(axes2)

    cubeAxesActor = vtk.vtkCubeAxesActor()
    cubeAxesActor.SetBounds((-3, 3, -3, 3, -2, 2))
    cubeAxesActor.SetCamera(ren.GetActiveCamera())
    cubeAxesActor.GetTitleTextProperty(0).SetColor(1.0, 0.0, 0.0)
    cubeAxesActor.GetLabelTextProperty(0).SetColor(1.0, 0.0, 0.0)
    cubeAxesActor.GetTitleTextProperty(1).SetColor(0.0, 1.0, 0.0)
    cubeAxesActor.GetLabelTextProperty(1).SetColor(0.0, 1.0, 0.0)
    cubeAxesActor.GetTitleTextProperty(2).SetColor(0.0, 0.0, 1.0)
    cubeAxesActor.GetLabelTextProperty(2).SetColor(0.0, 0.0, 1.0)
    cubeAxesActor.DrawXGridlinesOn()
    cubeAxesActor.DrawYGridlinesOn()
    cubeAxesActor.DrawZGridlinesOn()
    # if vtk.VTK_MAJOR_VERSION > 5:
    #     cubeAxesActor.SetGridLineLocation(vtk.VTK_GRID_LINES_FURTHEST)

    cubeAxesActor.XAxisMinorTickVisibilityOff()
    cubeAxesActor.YAxisMinorTickVisibilityOff()
    cubeAxesActor.ZAxisMinorTickVisibilityOff()
    # cubeAxesActor.GetProperty().SetColor(0, 255, 0)
    cubeAxesActor.GetXAxesLinesProperty().SetColor(0, 255, 0)
    cubeAxesActor.GetYAxesLinesProperty().SetColor(0, 255, 0)
    cubeAxesActor.GetZAxesLinesProperty().SetColor(0, 255, 0)
    ren.AddActor(cubeAxesActor)

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()

    style = vtk.vtkInteractorStyleSwitch()
    iren.SetInteractorStyle(style)
    style.SetCurrentStyleToTrackballCamera()

    def get_camera_info(obj, ev):

        if iren.GetKeyCode() == "s":
            w2if = vtk.vtkWindowToImageFilter()
            w2if.SetInput(renWin)
            w2if.Update()

            writer = vtk.vtkPNGWriter()
            writer.SetFileName("screenshot.png")
            writer.SetInputData(w2if.GetOutput())
            writer.Write()
            print "screenshot saved"

        # save to pdf
        if iren.GetKeyCode() == "s":
            exp = vtk.vtkGL2PSExporter()
            exp.SetRenderWindow(renWin)
            exp.SetFilePrefix("screenpdf")
            exp.SetFileFormat(2)
            exp.SetCompress(False)
            exp.SetLandscape(False)
            exp.SetSortToBSP()
            # exp.SetSortToSimple()  # less expensive sort algorithm
            exp.DrawBackgroundOn()
            exp.SetWrite3DPropsAsRasterImage(False)

    iren.AddObserver(vtk.vtkCommand.KeyPressEvent, get_camera_info, 1)
    iren.SetRenderWindow(renWin)
    renWin.Render()
    # ren.SetActiveCamera(camera)
    iren.Initialize()
    iren.Start()


def transform_grid(args):
    corner_arr = args[0]
    rot1 = args[1]
    rot2 = args[2]
    t1 = args[3]
    t2 = args[4]
    corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)
    return corners_in_pcd_arr[0]


def vis_ested_pcd_corners(ind=1):
    # pair_ind = 9
    pcd_result_file = "output/pcd_seg/" + str(ind).zfill(4) + "_pcd_result.pkl"
    csv_file = "pcd/" + str(ind).zfill(4) + ".csv"

    full_arr = np.genfromtxt(csv_file, delimiter=",", skip_header=1)

    grid_coords = generate_grid_coords()

    with open(os.path.abspath(pcd_result_file), "r") as f:
        pcd_result_ls = cPickle.load(f)
    assert pcd_result_ls is not None

    rot1 = pcd_result_ls[0]
    t1 = pcd_result_ls[1].reshape(1, 3)
    rot2 = pcd_result_ls[2]
    t2 = pcd_result_ls[3].reshape(1, 3)

    trans_grid_ls = []
    for coords in grid_coords:
        args = [[coord, rot1, rot2, t1, t2] for coord in coords[:3]]
        trans_coords = map(transform_grid, args)
        trans_coords.append(coords[3])
        trans_grid_ls.append(trans_coords)

    ren = vtk.vtkRenderer()
    ren.SetBackground(.2, .3, .4)
    ren.SetBackground(0.90196079, 0.96078432, 0.59607846)
    # ren.SetBackground(1., 1., 1.)

    for i in xrange(len(trans_grid_ls)):
        tmp_actor = draw_one_grd_vtk(trans_grid_ls[i])
        tmp_actor.GetProperty().SetOpacity(0.5)
        ren.AddActor(tmp_actor)

    show_only_marker = True
    if show_only_marker:
        marker_full_data_arr = exact_full_marker_data(csv_file, [pcd_result_ls[-1]])
        actor2 = vis_3D_points(marker_full_data_arr, color_style="intens_rg")
    else:
        actor2 = vis_3D_points(full_arr, color_style="intens_rg")
    ren.AddActor(actor2)

    transform2 = vtk.vtkTransform()
    transform2.Translate(0.0, 0.0, 0.0)
    axes2 = vtk.vtkAxesActor()
    axes2.SetUserTransform(transform2)
    ren.AddActor(axes2)
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.SetWindowName(str(i).zfill(4))
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    def get_camera_info(obj, ev):
        if iren.GetKeyCode() == "s":
            w2if = vtk.vtkWindowToImageFilter()
            w2if.SetInput(renWin)
            w2if.Update()

            writer = vtk.vtkPNGWriter()
            writer.SetFileName("screenshot.png")
            writer.SetInputData(w2if.GetOutput())
            writer.Write()
            print "screenshot saved"

    style = vtk.vtkInteractorStyleSwitch()
    iren.SetRenderWindow(renWin)
    iren.SetInteractorStyle(style)
    # style.SetCurrentStyleToTrackballActor()
    style.SetCurrentStyleToTrackballCamera()

    iren.AddObserver(vtk.vtkCommand.KeyPressEvent, get_camera_info, 1)

    iren.Initialize()
    renWin.Render()
    renWin.SetWindowName(str(ind).zfill(4))

    iren.Start()


def draw_chessboard_model(marker_size=marker_size):
    gird_coords = generate_grid_coords(x_res=marker_size[0], y_res=marker_size[1])
    grid_ls = [(p[0]).flatten()[:2] for p in gird_coords]
    corner_arr = np.transpose(np.array(grid_ls).reshape(marker_size[0], marker_size[1], 2)[1:, 1:], (1, 0, 2))
    c = np.zeros([corner_arr.shape[0], corner_arr.shape[1], 3]).reshape(
        corner_arr.shape[0] * corner_arr.shape[1], 3).astype(np.float32)
    c[0] = np.array([0, 0, 1])
    c[-1] = np.array([1, 0, 0])
    s = np.zeros(corner_arr[:, :, 0].flatten().shape[0]) + 20
    s[0] = 60
    s[-1] = 60

    plt.scatter(corner_arr[:, :, 0].flatten(), corner_arr[:, :, 1].flatten(), c=c, s=s)

    plt.plot(corner_arr[:, :, 0].flatten(), corner_arr[:, :, 1].flatten())

    plt.xlim(corner_arr[:, :, 0].min(), corner_arr[:, :, 0].max())
    plt.ylim(corner_arr[:, :, 1].min(), corner_arr[:, :, 1].max())
    plt.xlabel("x coordinates [cm]")
    plt.ylabel("y coordinates [cm]")
    # plt.axes().set_aspect('equal', 'datalim')
    plt.axis('equal')
    plt.show()


def convert_to_edge(file_name):
    # gray = cv2.imread('lines.jpg')
    gray = cv2.imread(file_name)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    return img


def find_latest(cali_file_ls):
    number_ls = []
    for file in cali_file_ls:
        tmp_ls = file.split("_")
        number_ls.append(ast.literal_eval(tmp_ls[0] + "." + tmp_ls[1]))
    return cali_file_ls[np.array(number_ls).argmax()]


def back_project_pcd(img, pcd_arr, color_arr, r_t, i, hide_occlussion_by_marker):
    # print pcd_arr
    rot_mat = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], r_t[2]),
                     np.dot(transforms3d.axangles.axangle2mat([0, 1, 0], r_t[1]),
                            transforms3d.axangles.axangle2mat([1, 0, 0], r_t[0])))
    transformed_pcd = np.dot(rot_mat, pcd_arr.T).T + r_t[3:]

    transformed_pcd_ls = pcd_arr.tolist()

    if not hide_occlussion_by_marker:  # whether remove occlussions by the chessboard
        if params["camera_type"] == "panoramic":
            pcd2angle_s = map(xyz2angle, transformed_pcd_ls)
            proj_pts = np.array(map(voxel2pixel, pcd2angle_s))
            point_s = 5
        elif params['camera_type'] == "perspective":
            intrinsic_paras_tuple = make_tuple(params['instrinsic_para'])
            intrinsic_paras = np.array(intrinsic_paras_tuple).reshape(3, 3)
            cam_coord_pcd = transformed_pcd.copy()

            # print cam_coord_pcd

            print "before filtering z: ", cam_coord_pcd.shape
            # cam_coord_pcd = cam_coord_pcd[np.where(cam_coord_pcd[:, 2] < 0)]
            # cam_coord_pcd = cam_coord_pcd[:20000, :]
            # print cam_coord_pcd

            inds = np.where(cam_coord_pcd[:, 2] > 0.2)
            cam_coord_pcd = cam_coord_pcd[inds]
            color_arr = color_arr[inds]
            # print cam_coord_pcd
            print "after filtering z: ", cam_coord_pcd.shape

            pcd_to_pix = (np.dot(intrinsic_paras, cam_coord_pcd.T)).T
            pcd_to_pix = pcd_to_pix[np.where(pcd_to_pix[:, 2] > 0)]
            proj_pts = (pcd_to_pix / pcd_to_pix[:, 2].reshape(-1, 1))[:, :2].astype(np.int16)

            point_s = 3
            # print proj_pts
            # 
            # print proj_pts.shape
        else:
            raise Exception("Camera type not correctly defined!")
    else:
        if params["camera_type"] == "panoramic":
            point_s = 5
        elif params['camera_type'] == "perspective":
            point_s = 3
        else:
            raise Exception("Camera type not correctly defined!")

        chessboard_result_file_path = "output/pcd_seg/" + str(i).zfill(4) + "_pcd_result.pkl"
        chessboard_result_file = cPickle.load(open(chessboard_result_file_path, "r"))
        rot1 = chessboard_result_file[0]
        t1 = chessboard_result_file[1].reshape(1, 3)
        # print "rot1*rot1.T: ", np.dot(rot1, rot1.T)
        rot2 = chessboard_result_file[2]
        t2 = chessboard_result_file[3].reshape(1, 3)
        corner_arr = chessboard_result_file[4].reshape(-1, 2)
        num = corner_arr.shape[0]
        corner_arr = np.hstack([corner_arr, np.zeros(num).reshape(num, 1)])

        rot_mat = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], r_t[2]),
                         np.dot(transforms3d.axangles.axangle2mat([0, 1, 0], r_t[1]),
                                transforms3d.axangles.axangle2mat([1, 0, 0], r_t[0])))
        trans_arr = np.zeros([4, 4])
        trans_arr[:3, :3] = rot_mat
        trans_arr[:3, 3] = np.array(r_t[3:])
        trans_arr[3, 3] = 1
        trans_matrix = np.asmatrix(trans_arr)

        corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)
        corners_in_pcd_arr = (trans_matrix[:3, :3] * np.asmatrix(corners_in_pcd_arr).T).T + trans_matrix[:3, 3].T
        corners_in_pcd_arr = np.array(corners_in_pcd_arr)

        # print "before removal: ", transformed_pcd.shape
        inds = remove_occlusion_of_chessboard(transformed_pcd, corners_in_pcd_arr)
        print "inds:", inds
        proj_pts = np.array(proj_pcd_2_pix(transformed_pcd))[inds].astype(np.int32)
        print "after removal: ", proj_pts.shape
        color_arr = color_arr[inds]

    print
    print proj_pts.shape[0], proj_pts.min(axis=0), proj_pts.max(axis=0)
    print
    for i in xrange(proj_pts.shape[0]):
        cv2.circle(img, (proj_pts[i][0], proj_pts[i][1]), point_s, tuple(color_arr[i].tolist()), -1)

    return img


def vis_back_proj(ind=1, img_style="edge", pcd_style="intens", hide_occlussion_by_marker=False):
    imgfile = "img/" + str(ind).zfill(params["file_name_digits"]) + "." + params['image_format']
    if img_style == "edge":
        gray = cv2.imread(imgfile)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    elif img_style == "orig":
        img = cv2.imread(imgfile)
    else:
        raise Exception("Please input the right image style")

    csvfile = "pcd/" + str(ind).zfill(params["file_name_digits"]) + ".csv"

    csv = np.genfromtxt(csvfile, delimiter=",", skip_header=1)
    pcd = csv[:, :3]
    dis_arr = np.linalg.norm(pcd, axis=1)
    intens = csv[:, 3]

    filels = os.listdir(".")
    cali_file_ls = []
    for file in filels:
        if file.find("cali_result.txt") > -1:
            cali_file_ls.append(file)
    if len(cali_file_ls) > 1:
        warnings.warn("More than one calibration file exit! Load the latest file.", UserWarning)
        latest_cali = find_latest(cali_file_ls)
        r_t = np.genfromtxt(latest_cali, delimiter=',')
        print "Load ", latest_cali, " as the extrinsic calibration parameters!"
    elif len(cali_file_ls) == 1:
        r_t = np.genfromtxt(cali_file_ls[0], delimiter=',')
        print "Load ", cali_file_ls[0], " as the extrinsic calibration parameters!"
    else:
        raise Exception("No calibration file is found!")

    if pcd_style == "intens":
        pcd_color = np.fliplr((cm.jet(intens.astype(np.float32) / intens.max()) * 255).astype(np.int32)[:, :3])
    elif pcd_style == "dis":
        pcd_color = np.fliplr((cm.jet(dis_arr / 10) * 255).astype(np.int32)[:, :3])
    else:
        print "Please input the right pcd color style"

    backproj_img = back_project_pcd(img, pcd, pcd_color, r_t, ind, hide_occlussion_by_marker)

    if max(backproj_img.shape[0], backproj_img.shape[1]) > 1000:
        resize_factor = 1000. / max(backproj_img.shape[0], backproj_img.shape[1])
        resized_img_for_view = cv2.resize(backproj_img, (0, 0), fx=resize_factor, fy=resize_factor)
    else:
        resized_img_for_view = backproj_img

    window_name = "ind: " + str(ind) + " img_style: " + img_style + " pcd_style: " + pcd_style + (
        " hide_occlusion " if hide_occlussion_by_marker else "")
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, resized_img_for_view)
    k = cv2.waitKey(0)
    if k == 27:  # wait for ESC key to exit
        cv2.destroyAllWindows()
    elif k == ord('s'):  # wait for 's' key to save and exit
        save_file_name = str(ind).zfill(params["file_name_digits"]) + "_" + img_style + "_" + pcd_style + (
            "_hide_occlusion" if hide_occlussion_by_marker else "") + "." + params['image_format']
        cv2.imwrite(save_file_name, img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        print "The image is saved to ", save_file_name
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # vis_back_proj(ind=1, img_style="orig", pcd_style="dis", hide_occlussion_by_marker=True)
    vis_back_proj(ind=1, img_style="edge", pcd_style="intens", hide_occlussion_by_marker=False)

    # vis_all_markers(np.arange(1, 5).tolist())
    # vis_all_markers([1])
    # vis_segments_only_chessboard_color(1)
    # vis_csv_pcd(ind=1)
    # vis_segments(ind=1)
    # vis_ested_pcd_corners(ind=1)
