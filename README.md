# Foosball Robot
This is a vision controlled foosball table. The camera above the table processes the image and tracks the ball's position. The Arduino receives that information via ROS communication and control six stepper motors simultaneously; linear and rotation axes for three bars.


<img width="500" alt="cameracallibrator.png" src="https://user-images.githubusercontent.com/52503908/104432977-6a055600-553e-11eb-997c-a28c9240fb2e.jpg">

# Demo
Dome video is here. &rarr; https://youtu.be/knL-YN4Qc_c

# Environment

## Hardware
- [Arduino Mega 2560](https://store.arduino.cc/usa/mega-2560-r3)
- [L6470 stepper motor driver](https://www.st.com/en/motor-drivers/l6470.html) ([datasheet](https://www.st.com/resource/en/datasheet/l6470.pdf))

### Circuit diagram

## Software

- Ubuntu 18.04
- ROS Melodic Morenia

### ROS packages

- [uvc_camera](http://wiki.ros.org/uvc_camera)
- [camera_calibration](http://wiki.ros.org/camera_calibration)
- [image_proc](http://wiki.ros.org/image_proc)
- [cv_brigde](http://wiki.ros.org/cv_bridge)
- [rosserial](http://wiki.ros.org/rosserial)
- [rqt_image_view](http://wiki.ros.org/rqt_image_view)
- ([Joy](http://wiki.ros.org/joy))

### System diagram
<img width="1000" alt="cameracallibrator.png" src="https://user-images.githubusercontent.com/52503908/104117345-fc9dcd80-5363-11eb-9976-a87af1b70c4d.png">


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

<img width="450" alt="cameracallibrator.png" src="https://user-images.githubusercontent.com/52503908/103461142-754acb80-4d5f-11eb-9abd-8c4442b3476c.png"> 

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

<img width="400" alt="image_raw.png" src="https://user-images.githubusercontent.com/52503908/103461149-81cf2400-4d5f-11eb-9fa3-825d1fed25ae.png"> <img width="400" alt="image_rect_color.png" src="https://user-images.githubusercontent.com/52503908/103461153-84317e00-4d5f-11eb-87c3-0e944ffca398.png"> 

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

### 2. Optimize HSV value

In order to track only the ball, you have to optimize the HSV value of the mask filter.  
To do so, save a camera view image using the save button of the rqt_image_viewer, and run `scripts/hsv.py` with the image.

```bash
$ python scripts/hsv.py [image file name]
```
Click the ball in the "hsv" image window, and check the 

<img width="1000" alt="hsv_optimization.png" src="https://user-images.githubusercontent.com/52503908/103461145-7c71d980-4d5f-11eb-9089-f0d71ea11f1a.png"> 
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

<img width="500" alt="hsv_optimization.png" src="https://user-images.githubusercontent.com/52503908/103473647-47f13280-4dde-11eb-940e-ca36cfdc0e00.gif"> 

### 3. Publishing ROS topic
Finally, publish the ball position as a ROS topic, which will be subscribed by the Arduino. `scripts/ball_position_publisher.py` conducts the entire image processing of the previous sections and publishing ROS topic `/ball_position (opencv_apps/Circle)`. This script will be used as the final form of this section.  
Fill the parameters in line 11~18 and confirm that the ball position is correctly published.

```bash
$ roslaunch launch/track_ball.launch
$ python scripts/ball_position_publisher.py
$ rostopic echo /ball_position
```

# 3. Let's play!
Now you've prepared all pieces. Kichoff is almost there!  

After writing "sketchbook/L6470_SPI_stepMoter_sketch/L6470_SPI_stepMoter_sketch.ino" into the Arduino Mega, launch the uvc_camera node and rosserial node by the following command.

```bash
$ roslaunch launch/stepper_camera_driver.launch
```
Once the nodes have been successfully launched, the motor positions will be initialized like this.

<img width="500" alt="hsv_optimization.png" src="https://user-images.githubusercontent.com/52503908/104434871-9d48e480-5540-11eb-8eaa-bd36699bf271.gif"> 

The last thing you have to do is launching the image processing node. If you start running this program, the whole ROS pipeline is establish and the robot starts moving.

```bash
$ python scripts/ball_position_publisher.py
```
Enjoy fighting!

<img width="500" alt="hsv_optimization.png" src="https://user-images.githubusercontent.com/52503908/104434386-04b26480-5540-11eb-88b3-14b09f56ff3b.gif"> 