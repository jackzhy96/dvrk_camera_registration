# dVRK hand-eye calibration package

A ROS package to perform camera registration for the dVRK robot.

**Setup notes**

This script was tested on ROS noetic and the dVRK release 2.1.0.

## Getting started

### Software setup
Before using this package make sure to setup the ros image pipeline as described in [here](docs/ros1_image_pipeline_setup.md). 

### Hardware setup
TODO


## Hand-eye calibration script

To perform the hand-eye calibration first attach the calibration aruco marker (TODO:**Add the marker and cube design to repo**) and run the [camera_registration.py](./camera_registration.py) script. This script first ask the user to move a the PSM to create a convex hull in which it is save to move and then move the robot automatically to collect data for the hand-eye calibration. 

After collecting the data, the script will generate a .json file with the transformation between the camera PSM and the camera.

Example arguments
```bash
python camera_registration.py  -a PSM2 -m 0.01  -c /jhu_daVinci/decklink/left/image_rect_color  -t /jhu_daVinci/decklink/left/camera_info
```

