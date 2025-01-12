# Group 7 Project Report

## Table of Contents
1. [Introduction](#introduction)
2. [ROS](#ros)
3. [Unity](#unity)
4. [Algorithm](#algorithm)

## Introduction

This report details the implementation of a robot arm pick-and-place project, utilizing Unity for visualization and target specification, ROS for robot configuration and communication, and OMPL (Open Motion Planning Library) with CHOMP (Covariant Hamiltonian Optimization for Motion Planning) as the motion planner. The UR10e robot, equipped with an RG2 gripper, serves as the robotic manipulator. The project demonstrates seamless integration between simulation and execution, involving Unity for publishing target and placement data, ROS for robot and service configuration, and Python for OMPL-based motion planning.

The pick-and-place operation is a fundamental task in robotics, often employed in manufacturing, logistics, and automation. This project integrates multiple tools and frameworks to simulate and execute a pick-and-place operation for the UR10e robot with the RG2 gripper. Unity facilitates intuitive visualization and input, while ROS serves as the middleware for robot configuration and communication. The OMPL library, enhanced with the CHOMP optimizer, is used for generating collision-free trajectories. RViz aids in visualizing the planned motion before execution in Unity.

## 2. Project Architecture

The project comprises three primary components:

- **Unity:**
  - Publishes coordinates for the pick-and-place targets.
  - Defines the polygon of the table placement area.
  - Includes an AI printer as an additional element in the workspace.

- **ROS:**
  - Hosts the URDF file of the UR10e robot with the RG2 gripper.
  - Configures custom services and messages for communication with Unity and OMPL.

- **OMPL and RPMstar:**
  - Codes motion planning logic in Python.
  - Connects with ROS to execute planning requests and visualize trajectories in RViz.


## ROS Configuration
The ROS is responsible for the below functionalities:
1. **URDF File Configuration:**
   - The URDF file of the UR10e robot is compiled.
   - The RG2 gripper is attached and configured with appropriate joint and end-effector definitions.

2. **Custom Service and Message Setup:**
   - Custom ROS services and messages are defined to:
     - Receive target and placement data from Unity.
     - Forward planning requests to the OMPL planner.
     - Return computed trajectories to Unity.

3. **RViz Integration:**
   - RViz is used for visualizing the robot’s planned trajectories.
4. **Moveit configuration:**
   - Generate moveit configuration for the use of planning.
<!-- 
### Implementation
*Details on how ROS was implemented in the project.* -->

### URDF file generations
**UR10e Robot:** The resource for the ur10e robot is at follow: https://github.com/ros-industrial/universal_robot

**RG2 Gripper:** The resource for the RG2 gripper is at follow: https://github.com/Osaka-University-Harada-Laboratory/onrobot

**The Step for UR10e_RG2 robot generations:**
The moveit configuration are genarated using moveit assistant. The steps are the following:
1. Load the URDF File:
   - Import the UR10e URDF file with the RG2 gripper attached.
   - Verify joint limits, end-effector configurations, and collision properties.
2. Define the Planning Group:
   - Create a planning group for the robot’s manipulator.
   - Include all relevant joints and links for the arm and gripper.
3. Setup the Robot Pose:
   - Specify default poses for the robot’s start and goal configurations.
4. Define End-Effector Settings:
   - Configure the RG2 gripper as the end-effector.
   - Set up gripper parameters like grasp width and actuation limits.
5. Test Configuration in RViz:
   - Launch RViz to validate the setup.
   - Perform preliminary motion planning tests to ensure compatibility.

### Custom Service and Message Setup:
~~~
# Service request
Ur10eMoveitJoints joints_input
geometry_msgs/Pose pick_pose
geometry_msgs/Pose place_pose

---
# Service response
#Ur10eTrajectory trajectories
moveit_msgs/RobotTrajectory[] trajectories
~~~
The above is the code for the MoverService.srv that receive the joint inputs, pick pose and place pose then return the trajectories for Unity to execute. It receive UniversalRobot.msgs as input for the request then return the Ur10eTrajectory.msg as the response.

~~~
float64[6] joints
geometry_msgs/Pose pick_pose
geometry_msgs/Pose place_pose
~~~
*The above is the UniversalRobotsJointsMsg.msg*
~~~
moveit_msgs/RobotTrajectory[] trajectory
~~~
*The above is the Ur10eTrajectory.msg*

### Moveit Configuration.
From the beginning tutorial (https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/pick_and_place), we reuse the moveit_msgs folder with the following service and messages.

<!-- Insert files uses and purpose for using -->

### Rviz Intergration

### Results
*Summary of the results obtained using ROS.*

Using ROS, we have succesfully generated the URDF files for our Ur10e_rg2 robot, 

## Unity
### Introduction
Unity is used to simulate the Ur10e_rg2 robot. With the ROS unity package, Unity is able to simulate the robotic arm needed for the project, and with SkecthFab in Unity package, we are able to load the 3d printer into the Unity scene and work from there.

### Setting up Unity scene
**Load URDF model and setup robot**

**Load 3d Printer**

<!-- **Building Msgs and Srv needed for planning** -->
**Setup ROS settings in Unity**

### ROS Unity Communication

1. **Publish Target**
2. **Publish Collision Objects**
3. **Publish Robot pose**
4. **Execute Trajectory**


### Results
*Summary of the results obtained using Unity.*

## Algorithm
### Introduction OMPL

The **Open Motion Planning Library (OMPL)** is a versatile framework for robotic motion planning. It offers an array of algorithms for solving high-dimensional pathfinding problems, with Probabilistic Roadmaps (PRM) and PRM* among its prominent methods.

The **PRM (Probabilistic Roadmap)** algorithm is a sampling-based planner that builds a graph or "roadmap" of collision-free configurations. It randomly samples points in the configuration space, connecting these points with feasible paths to form a network. During execution, the planner searches this network to identify a path from the start to the goal configuration. PRM is particularly effective in static environments with complex geometry.

The **PRM Star (Optimal Probabilistic Roadmap)** algorithm refines the standard PRM by introducing optimality criteria. Instead of simply finding any collision-free path, PRM Star focuses on minimizing a cost metric, such as path length or smoothness. It iteratively improves the roadmap connections, ensuring convergence towards an optimal solution as more samples are added. This makes PRM Star suitable for tasks requiring precision and efficiency, such as the pick-and-place operation in this project.


1. **Trajectory Planning Logic:**
   - Written in Python, the planner receives the start and goal configurations from ROS.
   - The PRM/ PRM Star algorithm computes smooth, collision-free trajectories within the defined constraints.

3. **Unity Execution:**
   - Validated trajectories are returned to Unity for real-time execution by the robot arm.


### Implementation OMPL PRM / PRM Star
*Details on how the algorithm was implemented in the project.*

### Results
*Summary of the results obtained using the algorithm.*
The integration of Unity, ROS, and OMPL with CHOMP successfully demonstrated:

- Accurate and collision-free motion planning for the UR10e robot.
- Seamless data exchange between Unity and ROS via custom services and messages.
- Realistic simulation of the pick-and-place operation with visual feedback in Unity and RViz.
- Dynamic adaptability to workspace constraints, including polygonal boundaries and obstacles.

### Conclussion and Future Work
This project highlights the potential of combining Unity, ROS, and OMPL for complex robotic tasks. Future enhancements may include:

- Adding real-time obstacle detection and avoidance using sensors.
- Integrating machine learning for adaptive planning and execution.
- Extending the project to include multi-robot coordination.

**References**

1. Unity Robotics Hub Documentation.
2. ROS Wiki: URDF, Services, and RViz.
3. OMPL Documentation: CHOMP Algorithm.
4. Universal Robots UR10e Technical Specifications.
5. OnRobot RG2 Gripper Documentation.