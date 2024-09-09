# ME5413 Autonomous Mobile Robot
Task details are in the file 'Homework 1_Perception.pdf'
## Homework1_Perception 

### Packages
python == 3.9
ROS1 == Noetic
opencv-python-headless==4.9.0.80

Package required for all systems:
numpy
opencv_contrib_python
opencv_python
matplotlib

Package required for Ubuntu system (Bonus Task):
rospy
sensor_msgs
std_msgs
And then install vision_msgs by typing these commands in the terminal:
sudo apt-get update
sudo apt-get install ros-noetic-vision-msgs


### 1. Task1 Single Object Tracking
You can find the result in the following folders: Homework1_Perception/Task 1/seq_{seq_number}

### 2. Task2 Multi Object Prediction
The trajectory results are shown in Jupter notebook.

### 3. Bonus Task Single Object Tracking in ROS
The Python code is in the following folder:me5413/catkin_ws/src/hw1_pkg\scripts

#### 3.1 First please source the workspace: source ~/catkin_ws/devel/setup.bash

#### 3.2 Run the following command to start the node:
roslaunch hw_1_pkg hw_1.launch
Note: You can set the sequence number in the 'hw1.launch' file.

#### 3.3 You can find the results(Rosbag) in the following folder:me5413/catkin_ws/src/hw_1_pkg/result
