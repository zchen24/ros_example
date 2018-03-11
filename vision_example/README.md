ecm vision pipeline 
=======================

### Folders & Files
* scripts
  * tune_camera.py: manually tune camera frame w.r.t. robot
  * move_psm.py: code to move psm around
* launch: 
  * mono.launch: launch mono cam 
  * stereo.launch: launch dvrk stereo cams
  * stereo_wam.launch: launch wam stereo cams 
* data: camera configuration files
  * jhu\_dvrk\.ini: dvrk stereo calib files 
  * wam\.ini: wam stereo calib files 
* matlab:
  * singleCamCapture.m: capture a bunch of imgs from a single cam
* src: some testing code using OpenCV & ROS 

