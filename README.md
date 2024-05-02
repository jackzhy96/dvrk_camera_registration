# dVRK hand-eye calibration package

A ROS package to perform camera registration for the dVRK robot.

**Setup notes**
This script was tested on ROS noetic and the dVRK release 2.2.1.
On Ubuntu 20.04 you might need to compile `gscam` (see notes in `dvrk_video` package). You might also have to install scipy with pip3 in your user directory using `pip3 install --user scipy==1.4.0`.

# Introduction

The goal of this package is to perform an hand-eye calibration between a dVRK PSM arm and the camera, ideally a stereo endoscope.  Most of the dVRK groups have an original endoscope and CCUs from Intuitive Surgical Inc with SDI outputs so we will assume a stereo camera even though this hand-eye calibration uses only one channel (mono).  The registration uses a small ArUco marker mounted on the shaft of the instrument (you can find an STL model for the mount in the `assets` directory).  The ArUco marker can be removed once the registration is performed.  The overall steps are:
* Calibrate the camera
* Run the calibration script
  * Manually move the PSM to safe locations to define the boundaries of the search space
  * The code will compute a convex hull based on the user provided poses
  * Let the code move the PSM around to collect data while not colliding with the environment
* Optionally, validate the results
* Copy/paste the resulting transformation to the dVRK console

# Steps

## Create a catkin package

You will need to create a new package to store the calibration results.  At that point, choose a name for your camera (aka stereo rig).  For this example, we will use `jhu_daVinci`:
```bash
cd ~/catkin_ws/src
catkin_create_pkg jhu_daVinci
catkin build
source ~/catkin_ws/devel/setup.bash # you have a new package, need to source
``` 

## Start the video

The following is based on dVRK provided launch files to capture the stereo video using Decklink SDI frame grabbers.  If your frame grabbers are different you will have to create your own launch files.  Note that you have to provided the name of your stereo rig:
```bash
roslaunch dvrk_video stereo_decklink_1280x1024.launch stereo_rig_name:=jhu_daVinci
```

## Calibrate the camera

The calibration is performed using the ROS provided program, please refer to their documentation for the parameters (http://wiki.ros.org/camera_calibration).  You  need to make sure the video stream is started and are using the correct rig name.
```bash
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 12x10 --square 0.0045 right:=/jhu_daVinci/right/image_raw left:=/jhu_daVinci/left/image_raw left_camera:=/jhu_daVinci/left right_camera:=/jhu_daVinci/right
```
Once the calibration is performed, don't forget to save and commit using the GUI.

## Restart the video using the calibration

At that point, you need to stop the launch file used for the video acquisition and restart it with the `stereo_proc` parameter set to `True`.  This will add a ROS node to compute the rectified images and camera parameters needed for the hand-eye registration.
```bash
roslaunch dvrk_video stereo_decklink_1280x1024.launch stereo_rig_name:=jhu_daVinci stereo_proc:=True
```

## Start the dVRK console

Start a dVRK console with the PSM you mean to register.

## Hand-eye calibration script

To perform the hand-eye calibration first attach the calibration ArUco marker and run the `camera_registration.py` script. This script first asks the user to move a the PSM to create a convex hull in which it is safe to move and then move the robot automatically to collect data for the hand-eye calibration. 
```bash
rosrun dvrk_camera_registration camera_registration.py -a PSM2 -m 0.01 -c /jhu_daVinci/left/image_rect_color -t /jhu_daVinci/left/camera_info
```

After collecting the data, the script will generate a couple of `.json` files with the transformation between the camera PSM and the camera.

## Validation script

We also provide a  script that overlays the estimated pose of the PSM end-effector based on the telemetry.
```bash
rosrun dvrk_camera_registration vis_gripper_pose.py -c /jhu_daVinci/left/image_rect_color -t /jhu_daVinci/left/camera_info -h PSM2_registration-open-cv.json
```

## Edit your `console.json`

The registration scripts also creates a `PSM<x>_registration-dVRK.json` file that contains a transformation you can copy/paste in your dVRK `console.json` to define the PSM `base-frame`.

