# dVRK hand-eye calibration package

A ROS package to perform camera registration for the dVRK robot.

**Setup notes**

This script was tested on ROS noetic and dVRK release 2.2.1+

On Ubuntu 20.04 you might need to compile `gscam` in your own catkin workspace (see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Camera-Calibration). You might also have to install scipy with pip3 in your user directory using `pip3 install --user scipy==1.4.0`.

# Introduction

The goal of this package is to perform hand-eye calibration between a dVRK PSM arm and the camera, ideally a stereo endoscope.  This can be especially useful for groups that don't have a full patient cart or a Setup Joint (SUJ) controller.  This script will tell you where the PSM is located with respect to the camera or ECM.  This will allow you to use the teleoperation components provided by the dVRK stack.  This code can also be used instead of the SUJ since the accuracy can be higher than the SUJs (SUJs are within 5cm cube reporting PSMs wrt ECM, the camera registration script seems to achieve an accuracy closer to 5mm to 10mm cube).
 
Most of the dVRK groups have an original endoscope and CCUs from Intuitive Surgical Inc with SDI outputs so we will assume a stereo camera even though this hand-eye calibration uses only one channel (mono).  The registration uses a small ArUco marker mounted on the shaft of the instrument (you can find an STL model for the mount in the `assets` directory as well as ArUco `.svg` files).  The ArUco marker can be removed once the registration is performed.  The overall steps are:
* Calibrate the camera
* Run the camera registration script
  * Manually move the PSM to safe locations to define the boundaries of the search space
  * The code will compute a convex hull based on the user provided poses
  * Let the code move the PSM around to collect data while not colliding with the environment
* Optionally, validate the results
* Copy-paste the resulting transformation to the dVRK console or your suj-fixed.json

There are two specific cases:
* Fixed camera.  In this case, the registration transformation can be used in your console.json to define the `base-frame`.
* Camera held by an ECM.  In this case, you should have a console.json that uses an SUJ of type `SUJ_Fixed`.  The SUJ has a "kinematic" configuration file  called `suj-fixed.json` by convention.  The result of the registration will need to be copy-pasted in this file.  Examples of configuration files using the Fixed SUJ can be found in https://github.com/dvrk-config/dvrk_config_jhu/tree/main/jhu-daVinci-Si.

The camera registration script will automatically save the correct transformation based on the `-e` command line option.  If you specify an ECM, the script assumes you are using a fixed SUJ.  Otherwise, it assumes you just need the PSM base frame for the console.json.

# Steps

## Camera calibration

See https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Camera-Calibration

## Start the dVRK console

Start a dVRK console with the PSM you mean to register.  If you have an ECM and want to register it as well, you need a console file with at least the PSM you want to register as well as the ECM.  We strongly recommend to use a console.json file with no base frame set for the PSM and ECM.  If you're using the Fixed SUJ, make sure the transformations in `suj-fixed.json` are all set to identity.

## Hand-eye calibration script

To perform the hand-eye calibration, you need to attach the calibration ArUco marker to the PSM shaft, ideally about 70mm from the tip.  The exact position doesn't matter.  You can 3D print the ArUco mount provided in the `assets` directory to hold the ArUco marker.  You will need to tap the smaller hole and use a screw to push against the instrument's shaft.

The `camera_registration.py` script first asks the user to move the PSM to create a convex hull in which it is safe to move.  During this stage, the PSM should be powered and you can use the clutch button to free the arm.  Move the tip without worrying if the ArUco marker is visible and try to create the largest volume possible in front of the camera.  When you're done, place the PSM so the ArUco marker is facing the camera and is close to the center of the volume you just created.  The script will then move the robot automatically to collect data for the hand-eye calibration.  To run the script:
```bash
rosrun dvrk_camera_registration camera_registration.py -p PSM2 -m 0.01 -c /jhu_daVinci/left
```
`-p` is used to indicate which PSM to use and `-c` is the namespace for your camera.  If you also have an ECM, add `-e ECM`:
```bash
rosrun dvrk_camera_registration camera_registration.py -p PSM2 -m 0.01 -c /jhu_daVinci/left -e ECM
```

After collecting the data, the script will generate a couple of `.json` files with the transformation between the PSM and the camera.  The file with `-open-cv` is used for the validation script below.

## Validation script

We also provide a  script that overlays the estimated pose of the PSM end-effector based on the telemetry.  You can run this script right after the camera registration.  Make sure the video is still running with `stereo_proc` set to `True`.
```bash
rosrun dvrk_camera_registration vis_gripper_pose.py -p PSM2 -c /jhu_daVinci/left -H PSM2-registration-open-cv.json
```

## Edit your dVRK configuration files

If you don't have an ECM (i.e., a fixed camera), the registration script creates a `PSM<x>_registration-dVRK.json` file that contains a transformation you can copy-paste in your dVRK `console.json` to define the PSM `base-frame`.  If you have an ECM, the content of `PSM<x>_registration-dVRK.json` needs to be copy-pasted in the kinematic file for the Fixed SUJ (aka `suj-fixed.json`). 
