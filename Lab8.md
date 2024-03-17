
# 💾 Lab 8: Mapping and Gazebo 
Sensors and Actuators (SEP 78s) Lab

By Moein Mehrtash (https://www.eng.mcmaster.ca/sept/faculty/dr-moein-mehrtash-lel/)

## GitHub Repository

{% embed url="https://github.com/septmacbot/macbot-deploy" %}

## Objectives
The objective of this laboratory session is to gain proficiency in utilizing OpenSLAM's GMapping algorithm for processing continuous
LiDAR point cloud data within a simulated environment using Gazebo and RVIZ. Through this exercise, participants will learn the intricacies
of integrating LiDAR data into a mapping algorithm, understanding the parameters involved in the GMapping algorithm, and visualizing the
mapping results in RVIZ. This hands-on experience aims to enhance understanding and practical skills in SLAM (Simultaneous Localization
and Mapping) techniques, crucial for various robotic applications and autonomous systems.


## Terminator as MacBot Application Launcher
Install the terminator application. The Terminator application in Ubuntu is a versatile and powerful terminal emulator that allows users to manage multiple terminal sessions within a single window. It provides features such as tabbed terminals, split terminals, customizable key bindings, and the ability to save and restore terminal layouts. Terminator enhances productivity by enabling users to work with different command-line interfaces simultaneously, facilitating efficient multitasking and organization of terminal workflows.. The application program lets you use multiple splitted and resized terminals, all at once on a single screen
```bash
sudo apt -y install terminator
```
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/76f3e23d-2a86-4fd5-9b10-c7facffbcc80)

Terminator is in the System Tools:

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/2fc1fb02-bbec-47a1-9068-5f3c83865c22)

You can also call it from another terminal 

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/91649123-37dc-40f3-9b18-04cb2587b45a)

Split your terminator layout into the following layout (7 Tabs). Right-click and select horizontal or vertical splits.
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/45f64636-53c2-43af-a3b4-b221dd233360)

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/72f513c2-7793-4c94-abbc-bbe5d2265b7d)


