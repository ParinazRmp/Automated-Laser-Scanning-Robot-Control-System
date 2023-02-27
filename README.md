# Simulation-of-ROS-robot-with-laser-scanner
This is a ROS-based robot simulation moving autonomously in a circular path equipped with a laser scanner to detect obstacles. You can use the controller to adjust the robot's velocity, stop it, and reset its position. 


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#Introduction">Introduction</a></li>
    <li><a href="#installing_and_running">Installing_and_running</a></li>
    <li><a href="#Project_Steps">Project_Steps</a></li>
    <li><a href="#Flowchart">Flowchart</a></li>
    <li><a href="#Simulation_and_Results">Simulation_and_Results</a></li>
    <li><a href="#Robot_Movement_Improvement_Proposal">Robot_Movement_Improvement_Proposal</a></li>
  </ol>
</details>



<!-- Introduction -->
## Introduction

This project involves simulating a robot's movement along a circular path using the Robot Operating System (ROS). 
The robot has a laser scanner that detects obstacles in front of it and provides distance measurements. 
The program allows you to control the robot's movement, including increasing or decreasing its linear velocity, stopping it, and resetting it to its starting position.
To complete this assignment, you will need to create a program using ROS and C++ that allows a robot to move around a circuit. 

The program should also include a service that provides a user interface for inserting different commands to change the behavior of the robot.




<!-- INSTALLING_and_RUNNING -->
## Installing_and_running

ROS is an open-source, meta-operating system for your robot.

[Here](https://wiki.ros.org/noetic/Installation/Ubuntu) is a guide for installing ROS on your machine.


* <h3>Creating a workspace for catkin: <h3>
```bash
mkdir -p RT1P/src
cd RT1P
catkin_make
```
* <h4>Running the Project: <h4>
```bash
cd src
```
```bash
git clone https://github.com/ParinazRmp/Automated-Laser-Scanning-Robot-Control-System.git
```
```bash
cd RT1P
```
```bash
catkin_make
```
	
I provided a file named ```.launch ```to enable the automatic running of the program.
```
roslaunch second_assignment second_assignment.launch
```



<!-- Project_Steps -->
## Project_Steps

### An overview of the project's steps

- Initially, a code was implemented to enable the robot to move autonomously within the environment. This involved creating a publisher and subscriber to change the robot's behavior based on its feedback.
- In the second step, a user interface was developed to allow keyboard inputs and modify the robot's velocity within the circuit. These changes were computed using a service that established communication between all nodes.

In the first step of the project, we created an autonomous controller for the robot. 

After starting the environment, we used the command `rostopic list` to check all running nodes. Then, we had to find the structure of each node of interest, such as `/base_scan` which provides data about the environment, and `/cmd_vel` which provides data about the velocity of the robot. We obtained the structure by using `rostopic info /base_scan` and `rostopic info /cmd_vel`.

At the end of this process, we also checked the structure of the data using the commands `rosmsg show sensor_msgs/LaserScan` and `rosmsg show geometry_msgs/Twist`.

It is evident that the `/base_scan` topic is the publisher, which provides data acquired using the laser in this case. 

On the contrary, the `/cmd_vel` is a subscriber, which computes the velocity of the robot and allows it to move. 

From the structure, it is clear that all fields of linear and angular velocity are among the three axes. According to the problem we are working on, the linear velocity can be on the *x* or *y-*axis, while the angular velocity can be on the *z-*axis only. Velocity on other axes, in this specific case, does not have any physical reason.

#### To determine the action the robot needs to take, it follows a simple logic:

- First, it takes all the sensors provided by the `/base_scan` topic and divides them into three main categories of equal dimensions:
    - Right array: sensors 0 to 109
    - Front array: sensors 304 to 414
    - Left array: sensors 609 to 719
- It then compares the distance on the right and left sides and makes the robot turn in the direction of the wall that is farthest away by changing its velocities. During a turn, the robot uses a small linear velocity on the *x* axis and an angular velocity on the *z* axis. Specifically:
    - An angular velocity < 0 turns the robot right
    - An angular velocity > 0 turns the robot left
- Finally, if the robot has a clear path without any nearby obstacles, it can adjust its velocity according to the menu provided through the *UI_node*.

<!-- Simulation_and_Results -->
## Simulation_and_Results

https://user-images.githubusercontent.com/94115975/218223479-2e771eb1-105f-4806-9850-aedf24090665.mp4

<!-- Robot_Movement_Improvement_Proposal -->
## Robot_Movement_Improvement_Proposal

The following report will outline two potential improvements to the current code. These modifications aim to enhance the robot's movement and make it smoother and more efficient.

Online control: The first improvement involves implementing online control to the robot's movement. The goal of this modification is to keep the robot centered and to prevent it from hitting walls while seeking the silver tokens. To achieve this, the robot will be designed to always see where the silver tokens are and maintain the center of the line.

Additional controls: The second improvement involves adding more controls to the code to handle different shapes of mazes. As the current code is straightforward and meets the requirements set by the professor, additional controls may be necessary to address any potential issues with the robot's movement in mazes with different shapes such as zig-zag or wave-shaped walls. These additional controls will help the robot navigate the maze more efficiently and effectively.

In conclusion, these two improvements have the potential to significantly enhance the robot's movement and make it smoother and more efficient. The implementation of online control and additional controls will allow the robot to better navigate the maze and perform its tasks more effectively.
