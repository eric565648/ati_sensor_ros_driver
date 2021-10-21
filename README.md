# ATIForceSensorROSDriver
The ROS package for ATI force torque sensor.

## System Requirement

1. ROS Noetic
2. Python3.8

## Run
Remeber to set up the workspace every time opening a new terminal.

In one terminal
```
roscore
```

In another terminal
```
rosrun ati_sensor_ros_driver ati_driver.py
```

If you want to visualize the wrench measure, 
1. open rviz.
2. Change the frame to `ati_frame`
3. Add -> By topic -> ati_ft

You should be seeing an pink arrow point to the total force direcion and a yellow arrow and circle illustrating the total torque.

## ROS Service

### Set tare
In one terminal
```
rosservice call /set_ati_tare
```

### Clear tare
In one terminal
```
rosservice call /clear_ati_tare
```

Generally you will not clear the tare.