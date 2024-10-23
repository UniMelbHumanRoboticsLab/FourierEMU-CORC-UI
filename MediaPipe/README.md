# FourierEMU-CORC-UI

## Upper-limb tracking

Set of Python scripts to perform an upper-limb pose tracking using MediaPipe and an Intel RealSense camera and to communicate to the main Unity program using FLNL to log the upper-limb pose in real-time.

### Dependencies
- Mediapipe: https://github.com/google/mediapipe 
- Intel RealSense camera package: https://pypi.org/project/pyrealsense2/
- RealSense camera access script: https://github.com/Razg93/Skeleton-Tracking-using-RealSense-depth-camera
- Keyboard package

In short ` pip install pyrealsense2 mediapipe keyboard`.


### Building executable
The main script `main.py` can be compiled to standalone executable using pyinstaller (tested with v5.2) and `main.spec` file:

```
pyinstaller main.spec
```

### Communication

The script starts an FLNL client responding to the following commands:
- "STA": Start streaming joint positions
- "STO": Stop streaming joint positions
- "DIS": Disconnect from server (and exit)
- "ARL": Switch to track Left arm
- "ARR": Switch to track Right arm

When streaming, the script regularly sends a value packet with:
- 0.0 or 1.0 for respectively Left or Right arm tracking
- x , y, z position in m for each of the tracked joints:

Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist for left arm tracking.

Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Right Elbow, Right Wrist for right arm tracking.


### Coordinate System 
The camera coordinate system has the origin at the center of RGB imager. The y-axis points from the imager out the lens. The x-axis is to the right in the image taken by the camera, and z is upward.

