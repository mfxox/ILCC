import numpy as np
print np
import cv2
import os
from ast import literal_eval as make_tuple
import config
import shutil
import sys

params = config.default_params()


# corner detection from one image
def get_corner_coords(imagefilename, backend=params['backend'], size=make_tuple(params['pattern_size']),
                      show_figure=False, save_figure=params['output_img_with_dectected_corners']):
    if backend == "matlab":
        try:
            import matlab.engine
            print "Matlab is used as backend for detecting corners"
        except ImportError:
            print "matlab.engine can not be found!"
            print "To use detectCheckerboardPoints function of matlab in python, matlab.engine for python should be installed!"
            sys.exit(0)

        eng = matlab.engine.start_matlab()
        imagePoints, boardSize, imagesUsed = eng.detectCheckerboardPoints(imagefilename, nargout=3)
        print boardSize, imagesUsed
        if not imagesUsed:
            print "Corners can not be detected!"
            return None

        np_imagePoints = np.array(imagePoints)
        if save_figure or show_figure:
            img = cv2.imread(imagefilename)
            size = tuple((np.array(boardSize).astype(np.int32) - 1).flatten())
            cv2.drawChessboardCorners(img, size, np_imagePoints.astype(np.float32), 1)
            if save_figure:
                save_imagefilename = os.path.join(params['base_dir'], "output")+"/img_corners/" + \
                                     (imagefilename.split("/")[-1]).split(".")[
                                         0] + "_detected_corners" + "." + params['image_format']
                cv2.imwrite(save_imagefilename, img)
                print "Image with detected_corners is saved in " + save_imagefilename
            if show_figure:
                cv2.imshow("image with detected corners", img)
                while True:
                    k = cv2.waitKey(1)
                    if k == 27:
                        cv2.destroyAllWindows()
                        break

        return np_imagePoints

    elif backend == "opencv":
        print "OpenCV " + str(cv2.__version__) + " is used as backend for detecting corners"
        img = cv2.imread(imagefilename)
        print img.shape

        ret, corners = cv2.findChessboardCorners(img, size,
                                                 flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
        #flags=cv2.cv.CV_CALIB_CB_ADAPTIVE_THRESH + cv2.cv.CV_CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        if not ret:
            print "Corners can not be detected!"
            return None

        cv2.drawChessboardCorners(img, size, corners, ret)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow('img', img)
        while True:
            k = cv2.waitKey(1)
            if k == 27:
                cv2.destroyAllWindows()
                break
        save_imagefilename = "output/img_corners/" + (imagefilename.split("/")[-1]).split(".")[
            0] + "_detected_corners" + "." + params['image_format']
        cv2.imwrite(save_imagefilename, img)
        return corners

    else:
        AssertionError("Please input the right backend for corner detection")


#
def detect_img_corners():
    ls = np.arange(1, 21).tolist()
    # ls = [20]
    img_corner_path = os.path.join(base_dir, "output/img_corners/")
    if os.path.isdir(img_corner_path):
        shutil.rmtree(img_corner_path)
    os.makedirs(img_corner_path)
    for i in ls:
        try:
            imagefilename = os.path.join(base_dir,
                                         "img", str(i).zfill(params['file_name_digits']) + "." + params['image_format'])
            print imagefilename
            corner_points = get_corner_coords(imagefilename)

            # print corner_points
            save_points_filename = img_corner_path + str(i).zfill(
                params['file_name_digits']) + "_img_corners" + ".txt"
            np.savetxt(save_points_filename, corner_points, delimiter=",")
        except:
            continue



if __name__ == '__main__':
    detect_img_corners()
