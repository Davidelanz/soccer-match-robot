# Four-wheeled Robot Design for Soccer Match

![ROS Distro](https://img.shields.io/badge/ROS-Melodic-Green)
![Raspberry Model](http://img.shields.io/badge/Raspberry-PI_3(Model_B)-red)
![Raspberry Model](http://img.shields.io/badge/Arduino-UNO-blue)

The aim of this project is to build a system able to play a robotic soccer match 1 vs 1. Subtasks involved:
  - Perception: detecting a red ball and (eventually) the opponent and the goal (blue)
  - Trajectory planning: controlling the velocity for reaching the target in an appropriate way
  - Task planning
  - Low level driver control

## Contents

1. [Guidelines](#guidelines)
1. [Scheme](#scheme)
1. [Getting started](#getting-started)
   1. [Download the ROS workspace](#download-the-ros-workspace)
   1. [Configure the Arduino motor driver](#configure-the-arduino-motor-driver)
   1. [Connect Raspberry's ROS master to remote workstation](#connect-raspberrys-ros-master-to-remote-workstation)
   1. [Launch the ROS node necessary for autonomous control](#launch-the-ros-node-necessary-for-autonomous-control)
   1. [Cameras](#cameras)
   1. [Control the robot with keyboard](#control-the-robot-with-keyboard)
1. [Troubleshoot](#troubleshoot)
1. [Authors](#authors)
   
   
## Guidelines

Sketch out the possible general architecture of the system, defining the main blocks constituting the model:
- Develop a ROS Module that recognize a red object and track it.
- Start testing your code in a simulation environment
- Take a look to:
   - [http://wiki.ros.org/cv_bridge]
   - [https://docs.opencv.org/3.3.1/d9/df8/tutorial_root.html]
   - [https://www.learnopencv.com/blob-detection-using-opencv-python-c/]

## Scheme

- L298 Motor drivers interface with the Arduino board:

<p align="center">
   <img src="https://github.com/Davidelanz/exrob_asmt2/blob/master/docu/L298N-Block-Diagram.png" alt="L298-With-Arduino" width="70%"/>
</p>

<p align="center">
   <img src="https://github.com/Davidelanz/exrob_asmt2/blob/master/docu/L298-With-Arduino.png" alt="L298-With-Arduino" width="70%"/>
</p>

- ROS Nodes architecture scheme (Raspberri Pi Robot & Remote Workstation):

![](https://github.com/Davidelanz/exrob_asmt2/blob/master/docu/scheme.svg?raw=True)

## Getting started

### Download the ROS workspace

Clone this repository on both the Raspberry Pi and the remote workstation:

```
https://github.com/Davidelanz/soccer-match-robot.git
```

Then build the workspace:

``` 
cd soccer-match-robot/
catkin_make
```

Remember to source it in the bash (for further details: http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

### Configure the Arduino motor driver

Simply load the `motor_driver.ino` sketch on the board. Remeber to choose the `Arduino due - programming port` board and `/dev/ttyACM0` port.

### Connect Raspberry's ROS master to remote workstation

> **Prerequisite**: the Raspberry must be connected on the same network of the remote workstation (e.g. a laptop).

1. Access the Raspberry via SSH by running in remote workstation's terminal. For example:
   ```ssh pi@raspberry.local```

1. Once got in Raspberry's terminal, export its IP address:

   ```pi@raspberry.local:~$ export ROS_IP=RASPBERRY_IP```
   
> If you need the machine's IP you can run  `hostname -I`

1. Launch the `raspicam` node in Raspberry's terminal (this command will also launch the `roscore`):

   ```pi@raspberry.local:~$ roslaunch raspicam_node camerav2_410x308_10fps.launch ```

1. Get back to the remote workstation and export its IP address as well:

   ```user@remote-workstation:~$ export ROS_IP=REMOTE_WORKSTATION_IP```

1. Then, connect the ROS master on the Raspberry to the remoting workstation by using the Raspberry's IP:
   
   ```user@remote-workstation:~$ export ROS_MASTER_URI=http://RASPBERRY_IP:11311/```


### Launch the ROS node necessary for autonomous control

1. In the Raspberry's terminal, launch the `motor_driver` node:

   ``` pi@raspberry.local:~$ rosrun motor_driver motor_driver.py ```

1. You can now launch the controller on another terminal of the Raspberry with:

   ```pi@raspberry.local:~$ rosrun bull_controller clever_bull_controller.py ```

1. In remote workstation's terminal run the red perception node for detecting the ball:

   ```user@remote-workstation:~$ rosrun robot_perception red_perception_node.py ```

1. In remote workstation's terminal run the `blue_perception` node for detecting the goal:
   
   ```user@remote-workstation:~$ rosrun robot_perception blue_perception_node.py ```

> Remember that you have to export the Raspberry's IP for each terminal you need to use and export the remote workstation's IP and connect it to the ROS master on the raspberry on each laptop's terminal


### Cameras

The ```raspicam_node``` supports dynamically reconfiguring the camera parameters.

- Run the dynamic reconfigure node on a connected computer:

``` rosrun rqt_reconfigure rqt_reconfigure ```

### Control the robot with keyboard

Pre-requisites: arduino sketch loaded and Raspberry's ROS master connected to the remote workstation

1. In Raspberry's terminal, launch the `motor_driver` node:

```pi@raspberry.local:~$ rosrun motor_driver motor_driver.py ```

- In remote workstation's terminal launch the `teleop_twist_keyboard` node and follow the instruction at screen:

```user@remote-workstation:~$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py ```

> Remember to increase the speed (`w` key) at least to 2/3 to get a decent feedback from the motor


## Troubleshoot:

SSH not enabled on Rasp:
   - [https://linuxize.com/post/how-to-enable-ssh-on-ubuntu-18-04/]
   - [https://askubuntu.com/questions/921258/how-to-enable-ssh-on-boot-in-ubuntu-mate]





## Authors
* Davide Lanza       - davidel96@hotmail.it
* Alberto Ghiotto    - alberto.ghiotto@hotmail.it
* Alberto Grillo     - albogrillo@gmail.com



EMARO Advanced Robotics Master - 2nd year
University of Genoa
