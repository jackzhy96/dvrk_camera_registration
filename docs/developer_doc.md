
# Notes

* change rig name to `jhu_daVinci_endoscope`
* send output to handeye to  `jhu_daVinci_endoscope`. This would be super cool.
* `pip install scipy`
* make opencv window smaller/resizeable.

* Fix error 

```bash
Exception ignored in: <function Image.__del__ at 0x7fef7267c670>
Traceback (most recent call last):
  File "/usr/lib/python3.8/tkinter/__init__.py", line 4017, in __del__
    self.tk.call('image', 'delete', self.name)
RuntimeError: main thread is not in main loop
Tcl_AsyncDelete: async handler deleted by the wrong thread
```


# Test commands

```
```

# Encountered errors

```
[ERROR] [1713557223.309248]: Unable to set camera info for calibration. Failure message: Error storing camera calibration.
```

# Problem encountered while refractoring original code


## Scipy errors

Currently testing the code with scipy 1.3.3

Error 1
```
File "camera_registration.py", line 183, in measure_pose
    rotation = np.float64(rotation_quaternion.as_matrix())
AttributeError: 'Rotation' object has no attribute 'as_matrix'
```
solution: https://github.com/scipy/scipy/issues/11685