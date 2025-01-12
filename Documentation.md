# Group 7 Project Report

## Table of Contents
1. [Introduction](#introduction)
2. [ProjectArchitecture](#2-project-architecture)
3. [ROS](#ros-configuration)
4. [Unity](#unity)
5. [Algorithm](#algorithm)
6. [Conclustion](#conclussion-and-future-work)

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

- **CHOMP:**
  - Codes motion planning logic in Python.
  - Connects with ROS to execute planning requests and visualize trajectories in RViz.

![Project Architecture](/Image/image.png)


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

### Launch File Addition
In the process of planning, we create a new launch file to run all the neccessary planning configuration.
> With the PlanTrajectory.launch file, we set the tcp_ip and tcp_port, create the server node for pkg ros_tcp_endpoint to facilitate Unity Ros communication, the node for our package ur10_e_rg2_moveit is also created with the file mover.py attached as the type
> 
> Then we inclue the launch of the file demo.launch to launch alongside our server. The demo.launch file play an intergal part to initialize other planning pipeline with the move_group.launch file.
~~~
<launch>
    <arg name="tcp_ip" default="0.0.0.0"/>
    <arg name="tcp_port" default="10000"/>
	
    <node name="server_endpoint" pkg="ros_tcp_endpoint"    type="default_server_endpoint.py" args="--wait" output="screen" respawn="true">
		<param name="tcp_ip" type="string" value="$(arg tcp_ip)"/>
        <param name="tcp_port" type="int" value="$(arg tcp_port)"/>
	</node>
    <node name="mover" pkg="ur10e_rg2_moveit" type="mover.py" args="--wait" output="screen"/>
    <include file="$(find ur10e_rg2_moveit)/launch/demo.launch" />
</launch>
~~~

## Unity
### Introduction
Unity is used to simulate the Ur10e_rg2 robot. With the ROS unity package, Unity is able to simulate the robotic arm needed for the project, and with SkecthFab in Unity package, we are able to load the 3d printer into the Unity scene and work from there.

### Setting up Unity scene
The majority of our Unity scene is reuse form Pick and Place tutorial with table enlarge in the X and Z scale too facilitate the size of the UR10e robot. Additionally, we load the UR10e robot the same way we would load the nyrio robot.

**Load 3d Printer**

For loading the 3D printer from sketchfab, we have to install the package sketchfab for Unity. After installation, we interact with the sketchfab tab, choose the option to import .gltf file and drag the .gltf file we downloaded to the window, choose the option using the current scene. The result is the succesfully imported printer.

<!-- **Building Msgs and Srv needed for planning** -->
**Setup ROS settings in Unity**

We build the neccessery Msgs and Srv that are needed for the project, first are the messages and services that are located inside the ur10e_moveit package. Then, we build the messages that existed in the moveit_msgs/ folder. Which are RobotTrajectory.msgs, CollisionObject.msgs.
### ROS Unity Communication
Unity and Ros communicate through the TCP_IP topics, for the 2 components to have communication, we have to set the IP address and port of the 2 components the same, then we execute the following task.

1. **Publish Collision Objects**
~~~
    public void PublishTable()
    {
        var tablePose = new PoseMsg
        {
            position = m_Table.transform.position.To<FLU>(),
            orientation = m_Table.transform.rotation.To<FLU>()
        };
        tablePose.position.z -= 0.4; 
        var tableCollisionObject = new CollisionObjectMsg
        {
            header = new HeaderMsg
            {
                frame_id = "world" 
            },
            id = "table",
            operation = CollisionObjectMsg.ADD, 
            primitive_poses = new List<PoseMsg> { tablePose }.ToArray(),
            primitives = new List<SolidPrimitiveMsg>
            {
                new SolidPrimitiveMsg
                {
                    type = SolidPrimitiveMsg.BOX,
                    dimensions = new double[] { 5, 5, 0.64 } 
                }
            }.ToArray()
        };

        m_Ros.Publish(m_TopicName1, tableCollisionObject);
        Debug.Log("Published table collision object.");
    }
~~~
2. **Publish Robot Joints**
>Code for publish joints into ROS
~~~
 public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();
        Debug.Log($"Pick Pose: Position: {CurrentJointConfig()}");
        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        Debug.Log($"Pick Pose: Position: {request.pick_pose.position.x}, {request.pick_pose.position.y}, {request.pick_pose.position.z} | Orientation: {request.pick_pose.orientation.x}, {request.pick_pose.orientation.y}, {request.pick_pose.orientation.z}, {request.pick_pose.orientation.w}");

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PlacePoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        Debug.Log($"Place Pose: Position: {request.place_pose.position.x}, {request.place_pose.position.y}, {request.place_pose.position.z} | Orientation: {request.place_pose.orientation.x}, {request.place_pose.orientation.y}, {request.place_pose.orientation.z}, {request.place_pose.orientation.w}");

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }
~~~
3. **Execute Trajectory**
> If the moverService return a trajectory, then the coroutine to execute trajectories would be run and the robot arm would move arcording to the trajectory.
~~~
void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }
~~~

## Algorithm
### Introduction CHOMP
The Covariant Hamiltonian Optimization for Motion Planning (CHOMP) is a gradient-based trajectory optimization algorithm that simplifies motion planning for robotic arms. Unlike traditional planners that separate trajectory generation into planning and optimization stages, CHOMP integrates these stages using covariant and functional gradient methods. Starting with an initial trajectory that may be infeasible, CHOMP adjusts it to avoid collisions while optimizing parameters such as joint velocities and accelerations. The algorithm quickly converges to a smooth, collision-free trajectory that can be executed efficiently by the robot. By framing motion planning as an optimization problem, CHOMP leverages the principles of the MMP framework, making it adaptable for high-dimensional tasks.

**Implement CHOMP**
~~~
<launch>
  <arg name="start_state_max_bounds_error" default="0.1" />
  <arg name="jiggle_fraction" default="0.05" />
  <!-- The request adapters (plugins) used when planning. ORDER MATTERS! -->
  <arg name="planning_adapters"
       default="default_planner_request_adapters/LimitMaxCartesianLinkSpeed
                default_planner_request_adapters/AddTimeParameterization
                default_planner_request_adapters/ResolveConstraintFrames
                default_planner_request_adapters/FixWorkspaceBounds
                default_planner_request_adapters/FixStartStateBounds
                default_planner_request_adapters/FixStartStateCollision
                default_planner_request_adapters/FixStartStatePathConstraints"
                />

  <param name="planning_plugin" value="chomp_interface/CHOMPPlanner" />
  <param name="request_adapters" value="$(arg planning_adapters)" />
  <param name="start_state_max_bounds_error" value="$(arg start_state_max_bounds_error)" />
  <param name="jiggle_fraction" value="$(arg jiggle_fraction)" />

  <rosparam command="load" file="$(find ur10e_rg2_moveit)/config/chomp_planning.yaml" />
</launch>
~~~
The above is the planning pipeline for CHOMP

>We set the default planning pipeline in move_group.launch into chomp
~~~
<!-- move_group settings -->
  <arg name="pipeline" default="chomp" />
  <arg name="allow_trajectory_execution" default="true"/>
  <arg name="moveit_controller_manager" default="simple" />
  <arg name="fake_execution_type" default="interpolate"/>
  <arg name="max_safe_path_cost" default="1"/>
  <arg name="publish_monitored_planning_scene" default="true"/>
~~~

>The planning pipeline 
~~~
    <include file="$(dirname)/planning_pipeline.launch.xml">
      <arg name="pipeline" value="chomp" />
    </include>
~~~

### Planning Process
1. **Trajectory Planning Logic:**
   - Written in Python, the planner receives the start and goal configurations from ROS.
   - The CHOMP algorithm computes smooth, collision-free trajectories within the defined constraints.

2. **Unity Execution:**
   - Validated trajectories are returned to Unity for real-time execution by the robot arm.


### Trajectory Planning Logic
> This code for plan_trajectory function take in the move_group current status, the destination pose and the starting point of the joint angle. We take the start angles, parse them into the robot state and set the start state for the move_group. Then we choose the set joint value target as the target pose and we call the plan function.
~~~
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_joint_value_target(destination_pose,True)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)


    return planCompat(plan)
~~~

>For planning pick and place process, first, we set the planner id to CHOMP to use under the OMPL library, the we set 5 poses for the grasping process. We plan each pose with the starting configuration or the previous pose planned as the starting joint vale. Finally, we append all the poses into a trajectory and publish them as a trajectory.
~~~
def plan_pick_and_place(req):
    scene = moveit_commander.PlanningSceneInterface()
    rospy.logwarn(scene.get_known_object_names())

    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planner_id("CHOMP")

    move_group.set_goal_tolerance(0.005) 
    move_group.set_planning_time(10)  
    current_robot_joint_configuration = req.joints_input.joints
    # publish_pose_as_axes(req.pick_pose, "pre_grasp_pose_topic")
    '''
    req.pick_pose.orientation.x = 0.5
    req.pick_pose.orientation.y = -0.5
    req.pick_pose.orientation.z = -0.5
    req.pick_pose.orientation.w = 0.5
    '''

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.32  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    
    if not grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    '''
    # Slightly move back

    req.pick_pose.position.x += 0.2
    slightly_move_back_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    if not slightly_move_back_pose.joint_trajectory.points:
        return response
    
    previous_ending_joint_angles = slightly_move_back_pose.joint_trajectory.points[-1].positions

    # Slightly move diagonal
    req.pick_pose.position.y -= 0.5
    req.pick_pose.position.z += 0.5
    slightly_move_diagional_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    if not slightly_move_diagional_pose.joint_trajectory.points:
        return response
    
    previous_ending_joint_angles = slightly_move_diagional_pose.joint_trajectory.points[-1].positions

    req.place_pose.position.x -= 0.1
    req.place_pose.position.y -= 0
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)
    # response.trajectories.append(drop_pose)

    move_group.clear_pose_targets()

    return response

~~~


### Results
The integration of Unity, ROS, and CHOMP successfully demonstrated:

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
3. CHOMP Documentation: CHOMP Algorithm.
4. Universal Robots UR10e Technical Specifications.
5. OnRobot RG2 Gripper Documentation.