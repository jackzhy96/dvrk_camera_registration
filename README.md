# dVRK hand-eye calibration package

Automatic hand-eye calibration package for the dVRK robot. Before using this package make sure to setup the ros image pipeline as described in [here](docs/ros1_image_pipeline_setup.md). 

**Setup notes**
This script was tested on ROS noetic and the dVRK release 2.1.0.

## Hand-eye calibration script

To perform the hand-eye calibration first attach the calibration aruco marker (TODO:**Add the marker and cube design to repo**) and run the [camera_registration.py](./camera_registration.py) script. This script first ask the user to move a the PSM to create a convex hull in which it is save to move and then move the robot automatically to collect data for the hand-eye calibration. 

After collecting the data, the script will generate a .json file with the transformation between the camera PSM and the camera.


## Trouble shooting. 

Many errors comming from scipy. Currently testing the code with scipy 1.3.3

Error 1
```
File "camera_registration.py", line 183, in measure_pose
    rotation = np.float64(rotation_quaternion.as_matrix())
AttributeError: 'Rotation' object has no attribute 'as_matrix'
```
solution: https://github.com/scipy/scipy/issues/11685