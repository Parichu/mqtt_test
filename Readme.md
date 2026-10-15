# How To Run This Code

### Step 1
Source the workspace environment (use bash or zsh according to your shell):
```bash
source install/setup.bash
```

### Step 2
Run the camera-to-map calibration node:
```bash
ros2 run mqtt_test cam_map
```

### Step 3
After the UI pops up, select at least 4 corresponding points between the Webcam and the Robot Map:
- Press **`c`** to enter Click Point mode.
- Click a point on the **Webcam** window, then click the corresponding point on the **Map (pgm)** window.
- Repeat for at least 4 points (spread evenly across the room).

### Step 4
After finishing point selection:
- Press **`h`** to compute the Homography Matrix that transforms the Webcam surface to the Robot Map coordinate frame.

### Step 5
Verify the calibration:
- Press **`v`** to toggle visualization and check that points from both surfaces align properly.

### Step 6
Save the calibration matrix:
- Press **`s`** to save the [`homography.json`](homography.json) file.
- This file is used by the YOLO detection and navigation nodes to steer the robot to goal points.

---

# Demo Video

[![Watch Calibration Demo Video](demo/thumbnail.jpg)](demo/Cam_Carlibrate.mp4)

> 🎥 **[Click here to open and watch the demo video (`demo/Cam_Carlibrate.mp4`)](demo/Cam_Carlibrate.mp4)**
https://github.com/user-attachments/assets/eec792ac-35a9-4fb8-8179-9d85909adc32
<video src="demo/Cam_Carlibrate.mp4" controls="controls" width="100%"></video>
