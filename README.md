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
- In the second step, a user interface was developed to allow keyboard inputs and modify the robot's velocity. These changes were computed using a service that established communication between all nodes.


### First Step	
In the first step of the project, we created an autonomous controller for the robot. 

After starting the environment, we used the command `rostopic list` to check all running nodes. Then, we had to find the structure of each node of interest, such as `/base_scan` which provides data about the environment, and `/cmd_vel` which provides data about the velocity of the robot. We obtained the structure by using `rostopic info /base_scan` and `rostopic info /cmd_vel`.

The `/base_scan` topic is the publisher, which provides data acquired using the laser in this case. 
The `/cmd_vel` is a subscriber, which computes the velocity of the robot and allows it to move. 

According to the problem we are working on, the linear velocity can be on the *x* or *y-*axis, while the angular velocity can be on the *z-*axis only. Velocity on other axes, in this specific case, does not have any physical reason.


- First, it takes all the sensors provided by the `/base_scan` topic and divides them into three main categories of equal dimensions:
    - Right array: sensors 0 to 109
    - Front array: sensors 304 to 414
    - Left array: sensors 609 to 719
- It then compares the distance on the right and left sides and makes the robot turn in the direction of the wall that is farthest away by changing its velocities. During a turn, the robot uses a small linear velocity on the *x* axis and an angular velocity on the *z* axis. Specifically:
    - An angular velocity < 0 turns the robot right
    - An angular velocity > 0 turns the robot left
- Finally, if the robot has a clear path without any nearby obstacles, it can adjust its velocity according to the menu provided through the *UI_node*.

### Second Step

In the second step, we implemented a controller that allows changing the velocity by inserting inputs from the keyboard. At the beginning of the project, there was only one node, the *circuitcontroller_node*, responsible for moving the robot. In this second step, we added two more nodes: one for the service, called *service_node*, and one for the user interface, called *UI_node*.

The general structure is as follows:

- *circuitcontroller_node*: makes the robot move;
- *UI_node*: takes the keyboard inputs;
- *service_node*: reads the value passed by the UI and determines, according to a switch case, the value to pass for changing the velocity.

All the files described above are put inside the *src* folder, as is usually done. Moreover, for the correct implementation of the program, there are still two files to be defined:

- the type of service;
- (optionally) a type of message to pass the value, which is nothing more than a format for casting the value passed.
- 

The general path to change the velocity is:

- UI_node
- service_node
- UI_node
- control_node

The program's communication structure can be found here:


![rosgraph](https://user-images.githubusercontent.com/94115975/221691781-e18807ed-5675-4067-9fed-122956ce7e87.png)
	

<!-- Simulation_and_Results -->
## Simulation_and_Results

https://user-images.githubusercontent.com/94115975/218223479-2e771eb1-105f-4806-9850-aedf24090665.mp4

<!-- Robot_Movement_Improvement_Proposal -->
## Robot_Movement_Improvement_Proposal

Possible improvements to the robot's performance include:

- Adding the ability to follow walls and avoid zigzag driving in certain situations.
- Optimizing the linear-to-angular velocity ratio during turns to prevent collisions.

Despite the potential for improvement, the robot's current performance is already satisfactory. A reasonable velocity multiplier (e.g., 2 or 1.5) allows the robot to complete multiple laps around the circuit without issue. Using excessively high multipliers (e.g., 3 or 2.5) increases the risk of wall collisions. Additional requirements may be necessary to improve the robot's movements, particularly at higher speeds.
