# 3D-LiDAR and panoramic camera 
<!-- based on reflectance intensity of the laser -->

[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)<br>
paper (available soon)
<!-- [[paper]](http://www.mdpi.com/journal/remotesensing)-->
## Dependencies
* Python >= 2.7.9
* [OpenCV](http://opencv.org/)
* [OpenGV](https://laurentkneip.github.io/opengv/page_installation.html)
* [Point Cloud Library (PCL)](http://pointclouds.org/)
* [PCL python bindings](<https://github.com/strawlab/python-pcl>)
* Other python packages: pip install -r [requirements.txt](requirements.txt)


## Optional
* [MATLAB engine for Python](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html): Corner detection from images with MATLAB
* [VTK](https://github.com/Kitware/VTK) >=7.0: 3D Visualization


## Usage
### Explanation of the files
```config.py```: parameter settings <br>
``` main.py```:  call other functions to process from the the corner detection of image and LiDAR to optimization for the final result <br>
```img_corners_est.py```: estimate corners of chessboard from images with OpenCV or MATLAB<br>
```pcd_corners_est.py```: estimate corners of chessboard from the point cloud<br>
```LM_opt.py```: load corresponding 2D-3D corners, calculate initial values with the PnP method, refine the result with LM method
```utility.py```: utility functions for various of visualization

### Extrinsic calibration
1. Modify ```config.py``` according to your situation.
2. Put acquired panoramic images (.png) and the corresponding point cloud (.csv file exported from Veloview) into the __img__ folder and __pcd__ folder respectively. The file should be named like 0001.png or 0015.csv. <br>
3. Run ```python main.py```. It will call ```img_corners_est.py``` to detect coordinates of image corners and ```pcd_corners_est.py``` to detect coordinates of point cloud corners into the __output__ folder. In addition, the segmentation results of the point cloud are also saved.  Then ```LM_opt.py``` is called to calculate the final extrinsic parameters.<br>
    - Coordinates of corners from images are saved to __output/img_corners__ with the filename *00XX_img_corners.txt* and images with marked corners are saved in the same folder with the file name *00XX_detected_corners.png* if _'output_img_with_dectected_corners'_ in '''config.py''' is set to __True__ .
    - Coordinates of corners from point clouds are save to __output/pcd_seg__ with the filename *00XX_pcd_result.pkl*.  Segments of each point cloud are output to __output/pcd_seg/00XX__.
    - The extrinsic calibration results are output in the end of the process and saved with the filename *YYYYMMDD_HHMMSS_calir_result.txt*.  Images of back-projected 3D points with the calculated parameters are saved to __output/__ if 'back_proj_corners' is set to **True**.

### Visualization ([VTK](https://github.com/Kitware/VTK) >=7.0 is necessary)
After the all results are saved with the aforementioned process, ```utility.py``` can be imported for visualization. 

* visualization of the point cloud from .csv file
```
    import utility
    utility.vis_csv_pcd(ind=1)
```
<div style="text-align: center">
<img src="readme_files/vis_csv.png" width = "50%" />
</div>

* visualization of the segmented results
```
    import utility
    utility.vis_segments(ind=1)
```
<div style="text-align: center">
<img src="readme_files/vis_seg.png" width = "50%" />
</div>

* visualization of the detected point cloud segment of the chessboard
```
    import utility
    utility.vis_segments_only_chessboard_color(ind=1)
```
<div style="text-align: center">
<img src="readme_files/vis_chessboard_only.png" width = "50%" />
</div>

* visualization of the detected point cloud segment of the chessboard and the estimated chessboard model
```
    import utility
    utility.vis_ested_pcd_corners(ind=1)
```
<div style="text-align: center">
<img src="readme_files/vis_est_marker.png" width = "50%" />
</div>

* visualization of all detected chessboards
```
    import utility
    utility.vis_all_markers(utility.vis_all_markers(np.arange(1, 21).tolist())
```
<div style="text-align: center">
<img src="readme_files/all_frames_side.png" width = "60%" />
<img src="readme_files/all_frames_top.png" width = "45%" />
</div>

<!---* Corner detection from images
* Corner detection from point clouds
* Optimization--->

## Example
### Sample Data
The sample data and results of detected corners can be downloaded from [here](https://www.dropbox.com/s/m0ogerftqav0fyx/ILCC_sample_data_and_result.zip?dl=0) (185.6M). <br>

The sample data are acquired with the [chessboard file](readme_files/chessboard_A0_0.75_6_8.pdf). 

[It](readme_files/chessboard_A0_0.75_6_8.pdf) contains 6*8 patterns and the size of the pattern is 7.5cm if it is printed by A0 size.

### Usage
```
git clone https://github.com/mfxox/ILCC
cd ILCC
wget https://www.dropbox.com/s/m0ogerftqav0fyx/ILCC_sample_data_and_result.zip
unzip ILCC_sample_data_and_result.zip
```
Then run ```python main.py```, the extrinsic calibration results will be saved.

<!-- ## To do
Integration for ROS -->

