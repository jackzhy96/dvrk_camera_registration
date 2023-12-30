# dVRK hand-eye calibration package



## Errors

Many errors comming from scipy. Currently testing the code with scipy 1.3.3


Error 1
```
File "camera_registration.py", line 183, in measure_pose
    rotation = np.float64(rotation_quaternion.as_matrix())
AttributeError: 'Rotation' object has no attribute 'as_matrix'
```
solution: https://github.com/scipy/scipy/issues/11685