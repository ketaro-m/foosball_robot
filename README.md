# Foosball Robot
description here...

# Demo

# Environment

- Ubuntu 18.04
- ROS Melodic Morenia
- [Arduino Mega 2560](https://store.arduino.cc/usa/mega-2560-r3)
- [Xbox 360 Wired Controller](https://www.amazon.com/Microsoft-Wired-Controller-Windows-Console/dp/B004QRKWLA)

## ROS packages

- [Joy](http://wiki.ros.org/joy)
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)

# Usage

Write "[L6470_SPI_stepMotor_sketch.ino](https://github.com/ketaro-m/foosball_robot/blob/joy/sketchbook/L6470_SPI_stepMoter_sketch/L6470_SPI_stepMoter_sketch.ino)" down into the Arduino Mega. Then run the launch file.

```
$ roslaunch (path to launch)/stepper_by_joystick_driver.launch
```

Once the ROS nodes have launched, you can control the foosball bars by operating the controller.

<img width="500" alt="fig1.png" src="https://github.com/ketaro-m/foosball_robot/blob/joy/img/joy_command.jpg"> 
 
