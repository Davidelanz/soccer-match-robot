# Eperimental Robotics Lab (EXROB) ROS workspace 

The aim of this project is to build a system able to play a robotic soccer match 1 vs 1
- Subtasks involved:
   - Perception: detecting a red ball,(eventually) the opponent and the goal (green)
   - Trajectory planning: controlling the velocity for reaching the target in an appropriate way
   - Task planning
   - Low level driver control

## Guidelines

Think about the possible general architecture of the system, defining the main blocks constituting your model.
- Develop a ROS Module that recognize a red object and track it.
- Start testing your code in a simulation environment
- Take a look to:
   - [http://wiki.ros.org/cv_bridge]
   - [https://docs.opencv.org/3.3.1/d9/df8/tutorial_root.html]
   - [https://www.learnopencv.com/blob-detection-using-opencv-python-c/]


### Load the motor driver arduino sketch
Simply load the motor_driver.ino sketch on the board. Remeber to choose the "Arduino due - programming port" board and "/dev/ttyACM0" port.

### Connect Raspberry's ros master to laptop ros node via SSH 
Prerequisite: the raspberry must be connected on the same network of the laptop

- Access the pi via SSH by running in laptop's terminal:

``` ssh pi@legobot.local ```

Password is ```carlo_conti```

- Once got in, in rasp's terminal, export the rasp ip with:

 ``` export ROS_IP=130.251.13.156 ```

- Launch the raspicam node in rasp's terminal by running:

``` roslaunch raspicam_node camerav2_410x308_10fps.launch ```

This command will also launch the roscore 

- Get back to the laptop and export its ip:

   - For laboratorium's wifi

``` export ROS_IP=130.251.13.134 ``` 


- Connect with the ros master on the pi with:
 	- Here you have to put the pi's ip

``` export ROS_MASTER_URI=http://130.251.13.156:11311/ ```


- If you need to get some machine's ip you can run

``` hostname -I ```


### Launch the rosnode necessary to allow the robot to move around autonomously


- In the pi's terminal, launch the motor driver node:

``` rosrun motor_driver motor_driver.py ```

- You can now launch the controller on another terminal of the raspberry with:

``` rosrun bull_controller bull_controller.py ```

- In laptop's terminal run the raspicam node and the perception node:



``` rosrun robot_perception red_perception_node.py ```


Note: remember that you have to export the raspberry's ip for each terminal you need to use and export the laptop's ip and connect it to the ros master on the raspberry on each laptop's terminal



### Cameras

The ```raspicam_node``` supports dynamically reconfiguring the camera parameters.

- Run the dynamic reconfigure node on a connected computer:

``` rosrun rqt_reconfigure rqt_reconfigure ```

### Control the robot with keyboard
After having loaded the arduino sketch and followed the instruction on how to Connect Raspberry's ros master to laptop ros node via SSH

- In the pi's terminal, launch the motor driver node:

``` rosrun motor_driver motor_driver.py ```

- In laptop's terminal launch the teleop_twist_keyboard node and follow the instruction at screen:

``` rosrun teleop_twist_keyboard teleop_twist_keyboard.py ```

Remember to increase the speed (w key) at least to 2/3 to get a decent feedback from the motor






### Troubleshoot:
ssh not enabled on Rasp 
   - [https://linuxize.com/post/how-to-enable-ssh-on-ubuntu-18-04/]
   - [https://askubuntu.com/questions/921258/how-to-enable-ssh-on-boot-in-ubuntu-mate]





## Authors
* Davide Lanza       - davidel96@hotmail.it
* Alberto Ghiotto    - alberto.ghiotto@hotmail.it
* Alberto Grillo     - albogrillo@gmail.com



EMARO Advanced Robotics Master - 2nd year
University of Genoa
