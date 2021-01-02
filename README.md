# Foosball Robot
description here...

# Demo

# Environment

## Hardware
- [Arduino Mega 2560](https://store.arduino.cc/usa/mega-2560-r3)
- [L6470 stepper motor driver](https://www.st.com/en/motor-drivers/l6470.html)

### Circuit diagram

## Software

- Ubuntu 18.04
- ROS Melodic Morenia

###  ROS packages

- [uvc_camera](http://wiki.ros.org/uvc_camera)
- [camera_calibration](http://wiki.ros.org/camera_calibration)
- [image_proc](http://wiki.ros.org/image_proc)
- [rqt_image_view](http://wiki.ros.org/rqt_image_view)
- [Joy](http://wiki.ros.org/joy)

# 1. Camera Calibration

## ROS Wiki
- [camera_calibration](http://wiki.ros.org/camera_calibration)
- [How to Calibrate a Monocular Camera](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)


First, download the checker board ([check_7x6_27mm.pdf](http://wiki.ros.org/turtlebot_kinect_arm_calibration/Tutorials/CalibratingKinectToTurtleBotArm?action=AttachFile&do=view&target=check_7x6_27mm.pdf)) and run the calibration program. Move the checker board around in the camera view.

```bash
$ ls /dev/ | grep video # check the device name

# run calibration programs (in different terminals)
$ roscore
$ rosrun uvc_camera uvc_camera_node _device:=/dev/video1
$ rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.027 image:=/image_raw
```

<img width="400" alt="cameracallibrator.png" src="https://user-images.githubusercontent.com/52503908/103461142-754acb80-4d5f-11eb-9abd-8c4442b3476c.png"> 
<br />
<br />

Then extract the gz file and place the yaml file in the appropriate directory.

```bash
# make camera.yaml
$ mkdir camera_calibration; cd camera_calibration
$ mv /tmp/calibrationdata.tar.gz ./
$ tar -xvzf calibrationdata.tar.gz
$ mv ost.yaml camera.yaml
$ vi camera.yaml 
# rewrite "camera_name: narrow_stereo" -> "camera_name: camera", 
#         "camera_model: plumb_bob" -> "distortion_model: plumb_bob"
```

Finally, check the result

```bash
$ roscore
$ rosrun uvc_camera uvc_camera_node _device:=/dev/video1 _camera_info_url:=file://$PWD/camera_calibration/camera.yaml
$ rosrun image_proc image_proc
$ rqt_image_view
```

or run this launch file (yet, need to modify the absolute path to camera.yaml in line 4)

```bash
$ roslaunch launch/calibration_demo.launch
```

<img width="300" alt="image_raw.png" src="https://user-images.githubusercontent.com/52503908/103461149-81cf2400-4d5f-11eb-9fa3-825d1fed25ae.png">
<img width="300" alt="image_rect_color.png" src="https://user-images.githubusercontent.com/52503908/103461153-84317e00-4d5f-11eb-87c3-0e944ffca398.png"> 
<br />
<br />

# 2. Ball Tracking

### 1. Check the pixels of the fields.

After establishing the camera nodes, open rqt_image_view and select /image_rect_color. Then click the top-left and bottom-right points and check their pixel x, y values with echoing /image_rect_color_mouse_left.

```bash
$ roslaunch launch/track_ball.launch
$ rqt_image_view # select /image_rect_color
$ rostopic echo /image_rect_color_mouse_left
```
Fill these values in `scripts/hsv.py` and `scripts/track_ball_demo.py`.

```Python
field_area = [[33, 50], [605, 405]] # [[top-left x,y], [bottom-right x, y]]
```
<br />

### 2. Optimize HSV value

In order to track only the ball, you have to optimize the HSV value of the mask filter.  
To do so, save a camera view image using the save button of the rqt_image_viewer, and run `scripts/hsv.py` with the image.

```bash
$ python scripts/hsv.py [image file name]
```
Click the ball in the "hsv" image window, and check the 

<img width="1000" alt="hsv_optimization.png" src="https://user-images.githubusercontent.com/52503908/103461145-7c71d980-4d5f-11eb-9089-f0d71ea11f1a.png"> 
<br />
<br />

Lastly, to check if the ball is correctly tracked, fill the hsv_lower and hsv_upper values obtained above and run `scripts/track_ball_demo.py`. Looking at the result, you might have to modify hsv values and filter sizes of image processings.

(track_ball_demo.py)

```Python
# obtain from hsv.py
hsv_lower = np.array([20, -10, 100])
hsv_upper = np.array([50, 64, 300])
median_size = 7      # filter size for median filter
morpho_size = 13     # filter size for morphology processing
```
In terminal, run the following commands.

```bash
$ roslaunch launch/track_ball.launch
$ python scripts/detect_ball_demo.py
```

<img width="500" alt="hsv_optimization.png" src="https://user-images.githubusercontent.com/52503908/103461075-c1494080-4d5e-11eb-8fd4-255972b20b75.gif"> 
<br />
<br />

### 3. 