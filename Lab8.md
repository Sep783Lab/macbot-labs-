
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

## Gazebo and Unified Robot Description Format
Gazebo is a ROS graphical simulation tool. It includes packages to aid in the simulation of forces and physics including gravity, torque, etc. Gazebo brings a fresh approach to simulation with a complete toolbox of development libraries and cloud services to make simulation easy. Iterate fast on your new physical designs in realistic environments with high fidelity sensors streams. Test control strategies in safety, and take advantage of simulation in continuous integration tests. In Gazebo, the URDF (Unified Robot Description Format) file plays a central role in defining the physical characteristics and kinematic structure of a robot model used for simulation.

URDF stands for Unified Robot Description Format (http://wiki.ros.org/urdf). It is an XML-based file format used in ROS (Robot Operating System) to describe the physical and kinematic properties of robots, including their geometry, joints, sensors, and other attributes. URDF files play a crucial role in robot modeling, simulation, and visualization within the ROS ecosystem. Here are the key components typically found in a URDF file:

- Robot Model: The URDF file contains the geometric description of the robot, including links (visual and collision properties) and their respective transformations.

- Joints: Joints define the kinematic structure of the robot, specifying how links are connected and how they can move relative to each other. URDF supports various joint types such as revolute, continuous, prismatic, fixed, etc.

- Inertial Properties: URDF files include inertial properties for each link, such as mass, inertia matrix, and center of mass. These properties are essential for simulating the dynamics of the robot accurately.

- Visual and Collision Geometry: Each link in the URDF file can have both visual and collision geometry. Visual geometry defines the appearance of the link in visualizations, while collision geometry is used for collision detection in simulations.

- Sensors: URDF files can also describe sensors attached to the robot, such as cameras, lidars, IMUs, etc. Sensor parameters such as field of view, resolution, and noise characteristics can be specified.

- Transmission and Actuators (optional): For robots with actuators, URDF files may include transmission tags to define how joint movements are translated into actuator commands. This is useful for simulating actuated joints accurately.

- Robot Description: The complete URDF file provides a comprehensive description of the robot's physical properties, kinematics, dynamics, and sensor configurations. This information is crucial for various robotics applications, including simulation, motion planning, and control.

  Here is of an example of two links robot:
  ![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/1083939d-1fbf-4475-a3bd-2cc1a68f113b)

    ```xml
      <?xml version="1.0"?>
    <robot name="two_link_robot">
    
      <!-- Define the first link -->
      <link name="link1">
        <visual>
          <geometry>
            <box size="0.1 1.0 0.1"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="0.1 1.0 0.1"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0.5"/>
          <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
        </inertial>
      </link>
    
      <!-- Define the second link -->
      <link name="link2">
        <visual>
          <geometry>
            <box size="0.1 1.0 0.1"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="0.1 1.0 0.1"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <origin xyz="0 0 0.5"/>
          <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
        </inertial>
      </link>
    
      <!-- Define the first joint -->
      <joint name="joint1" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57"/>
      </joint>
    
    </robot>
    ```
The URDF file (robot_textured.xacro) of the Macbot is located here: `~/macbot/macbot_ws/src/macbot_ros/macbot_description/urdf`.

```bash
gedit  ~/macbot/macbot_ws/src/macbot_ros/macbot_description/urdf/robot_textured.xacro
```

1. In the top left terminal emulator, launch ROSCore. Ensure it launches before proceeding.
```bash
    roscore
```
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/f06a1871-e653-4875-b0e0-6b5d657feb3d)

2. In the bottom left terminal emulator, launch the following file (`~/macbot/macbot_ws/src/macbot_ros/macbot_gazebo/launch/default.launch`). Ensure that it launches successfully before proceeding.
```bash
    roslaunch macbot_gazebo default.launch 
```

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/a3e5169b-0986-4284-bad6-f22849474d0e)


This launch file utilizes the URDF format to generate the robot model in Gazebo. You can examine the model's structure in the left column for further details.

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/e4281823-1634-4429-90a2-9b86d24b5cee)

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/88839955-3a30-400e-bc9c-31e28cf9b386)


This launch file also launches the RViz application with the robot model, including a lidar sensor.
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/548585e7-f97b-4747-bb4c-8d4155d574e8)


3. Close RViz and Gazebo (File\Quit). Terminate the launch node using Terminator ( CTL+x and CTL+c). Then, launch Gazebo with a 'maze' environment by running:
  
```bash
    roslaunch macbot_gazebo default.launch world:=maze 
```
This launch file include 'maze' world to the model in Gazebo.
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/91ea3714-2cdc-481c-816f-6b3b713897ea)

Look in RViz that the laser scanner (LiDAR) is scanning the environment in front of the robot.
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/37a6fc80-62ac-4b5a-a0ff-a092dd927c36)


4. Close RViz and Gazebo (File\Quit). Terminate the launch node using Terminator ( CTL+x and CTL+c). Then, launch Gazebo with a 'maze' environment and RVIZ for mapping:

```bash
    roslaunch macbot_gazebo default.launch world:=maze rviz_config:=mapping
```
![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/f304fffd-870b-4f26-83af-275d8cc7d46a)

This will load the RVIZ in wireframe to use less memory for rendering. Notice that your RVIZ visualization data are in an error state. This is because RVIZ isn’t yet being provided with simulated point-cloud data.

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/3d5f8631-fce8-4b3b-8374-6f36899fff75)


## gmapping package in ROS 

The `gmapping` package in ROS (Robot Operating System) provides functionality for SLAM (Simultaneous Localization and Mapping) (https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) using the GMapping algorithm. SLAM is a fundamental capability for mobile robots to navigate and map their surroundings autonomously. Here's an overview of the functionality provided by the gmapping package:

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/fc77ef51-0435-450b-bf83-bf4b25f55c07)
2005 DARPA Grand Challenge winner Stanley performed SLAM as part of its autonomous driving system.



- SLAM Algorithm: The core functionality of the gmapping package is the GMapping algorithm, which performs SLAM by simultaneously estimating the robot's pose (location and orientation) and creating a map of its environment using sensor data, typically from a laser scanner (LIDAR).

- Map Creation: GMapping generates a 2D occupancy grid map that represents the environment around the robot. The map indicates which areas are occupied, free, or unknown based on sensor measurements and robot movement.

- Pose Estimation: The algorithm continuously estimates the robot's pose relative to the map as it moves, incorporating sensor measurements and odometry data to update the pose estimate.

- Loop Closure Detection: GMapping includes mechanisms for loop closure detection, which helps in correcting accumulated errors in pose estimation over time. Loop closures occur when the robot revisits a previously visited location, allowing the algorithm to refine the map and pose estimates.

- ROS Nodes: The gmapping package provides ROS nodes that can be used to integrate SLAM capabilities into ROS-based robotic systems. Key nodes include:
    slam_gmapping: Responsible for running the GMapping algorithm and generating the map.
    map_server: Saves and loads map files generated by GMapping.
    move_base: Integrates SLAM-generated maps with navigation and path planning for robots.
- Integration with ROS Navigation Stack: GMapping is often used in conjunction with other packages in the ROS Navigation Stack, such as move_base, amcl (Adaptive Monte Carlo Localization), and costmap_2d, to enable robots to navigate autonomously in known environments.

- Dynamic Map Updates: GMapping supports dynamic map updates, allowing the map to be updated in real-time as the robot explores its environment, detects new obstacles, or revisits previously mapped areas.

- Parameter Tuning: Users can adjust various parameters of the GMapping algorithm, such as sensor characteristics, map resolution, particle filter settings, and loop closure thresholds, to optimize performance for specific robot platforms and environments.

Overall, the gmapping package in ROS is a critical component for enabling robots to build maps of their surroundings and localize themselves within those maps, facilitating autonomous navigation and exploration tasks.

In one of the treminal launch the gmap ( the luanch file located: `~/macbot/macbot_ws/src/macbot_ros/macbot_navigation/launch/gmap.launch`)
  
```bash
    roslaunch macbot_navigation gmap.launch
```
After a few moments, in RVIZ you should be able to see a map being generated.

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/91ea6e8d-bc21-48e3-ac96-35a6af5a6e13)

![image](https://github.com/Sep783Lab/macbot-labs-/assets/150700633/37481a80-4170-4c70-9a81-3bfe25de3f43)







