# Foosball Robot
description here...

# Demo

# Environment

- Ubuntu 18.04
- ROS Melodic Morenia

## ROS packages

- [uvc_camera](http://wiki.ros.org/uvc_camera)
- [camera_calibration](http://wiki.ros.org/camera_calibration)
- [Joy](http://wiki.ros.org/joy)

# Camera Calibration

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

<img width="600" alt="cameracallibrator.png" src="https://github.com/ketaro-m/foosball_robot/blob/camera/img/cameracalibrator.png"> 

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
```
$ roslaunch launch/calibration_demo.launch
```

<img width="600" alt="image_raw.png" src="https://github.com/ketaro-m/foosball_robot/blob/camera/img/image_raw.png"> 
<img width="600" alt="image_rect_color.png" src="https://github.com/ketaro-m/foosball_robot/blob/camera/img/image_rect_color.png"> 