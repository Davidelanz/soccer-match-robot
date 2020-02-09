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

### Connect to Raspberry via SSH 

- After having setup the vnc server 
   - [https://www.digitalocean.com/community/tutorials/how-to-install-and-configure-vnc-on-ubuntu-18-04]

- Run in laptop's terminal:

``` ssh rasp@192.168.43.18 ```

Password is "rasp"

- Then launch the vnc server from the pi's root with:

``` vncserver ```

- Logout from the rasp's root and setup the vnc connection between laptop and pi by inserting in the laptop's terminal:

``` ssh -L 5901:127.0.0.1:5901 -C -N -l rasp 192.168.43.18 ```

- Then open a vnc client and connect to 

``` localhost:5901 ```


ps: working on using only SSH access without a VNC server

### Launch the rosnode necessary to remotely control the robot

- Once logged in with your vnc client, open a terminal and run:

```roscore ```

- In another terminal launch the motor driver node:

``` rosrun motor_driver motor_driver.py ```

- In another terminal launch the teleop_twist_keyboard node and follow the instruction at screen:

``` rosrun teleop_twist_keyboard teleop_twist_keyboard.py ```

remember to increase the speed (w key) at leat to 2/3 to get a decent feedback from the motor 






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
