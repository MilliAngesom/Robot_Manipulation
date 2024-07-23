# Robotic Manipulation

This project aims to enable a robot to autonomously detect objects using ARUCO markers attached to them and utilize its robotic arm to move the objects to the desired destination.

## Members of the goup:

This project has been carried out by:

* Huy Nguyen
* Million Angesom
* Mohsin Kabir

## Prerequisites:
- **ROS Neotic**: Ensure ROS Neotic is installed. Other ROS versions are also fine.
    ```bash
    export CATKIN_WS=~/catkin_ws/  # Set your Catkin workspace environment variable appropriately.
    ```
- **Stonefish Simulator**: Install Stonefish for simulation capabilities. Follow instructions at [Stonefish Documentation](https://stonefish.readthedocs.io/en/latest/install.html). After installation, clone and compile the ROS interface:
    ```bash
    git clone https://github.com/patrykcieslak/stonefish_ros ~/catkin_ws/src/
    ```
- **Additional Dependencies**: Follow installation guides for additional required packages in [turtlebot_simulation](https://bitbucket.org/udg_cirs/turtlebot_simulation/src/master/)
## Usage:

To execute the project file, first open a command prompt and run the following commands:


```roslaunch turtlebot_simulation turtlebot_hoi.launch```

This command will launch the stonefish and rviz for the simulation.

In a new terminal, navigate to the directory containing the Python script and run the following commands to execute the code:

```python3 aruco_detect_intervention.py```

This command will launch the aruco detection node. Then in other terminal, run this command to execute the task priority implementation.

```python3 task_priority_intervention.py```

Finally, run the behavior tree to employing the project via this command:

```python3 intervention_behavior_tree.py```

The script will start controlling the mobile manipulator robot based on the defined tasks and inputs from the aruco detection, slam and robot joint states.

## Functionality:
The code performs the following tasks:
1. *aruco_detect_intervention.py*  detecting ArUco markers using a camera in a ROS environment
    * Initializes the necessary variables, subscribes to the camera topics, and sets up the transformation matrix.
    * Extracts the camera calibration parameters from the message.
    * Processes the image, detects ArUco markers, calculates their pose, and publishes the result.

2. *task_priority_intervention.py* 
    * Initializes the necessary variables and objects.
    * Sets up ROS publishers, subscribers, and service clients for communication with other ROS nodes.
    * Defines tasks related to the robot's end-effector position, configuration, and orientation.
    * Implements a task priority algorithm to control the robot's motion based on the defined tasks.
    * Publishes the computed velocities to control the robot's joints and base.
    * Monitors the error between the actual and desired end-effector pose and repeats the task priority algorithm until the error is below a threshold.

3. *kinematic_intervention.py* includes several classes and functions related to robotic manipulators
    * Performing the damped least-squares (DLS) solution to the matrix inverse problem.
    * MobileManipulator represents initialization and functions with joint angles and other parameters.
    * Task class initializes the task with a name and a desired value as well as other parameters.
    * Position3D, Configuration3D, Orientation, BaseOrientation, JointLimits3D, JointPosition3D, JointLimit are subclasses of the Task class.

4. *intervention_behavior_tree.py* implements a behavior tree for a pick and place task using the py_trees library. The behavior tree consists of several behaviors that represent different actions in the task, such as moving to specific points, picking an object, and placing an object.



## Video Demo
1. [Simulation Video](https://youtu.be/Saj-s225QKM)
2. [Real Robot Demo](https://youtu.be/9XrvEi3ctUM)

## Paper
The detailed explanation of the project can be accessed [here](https://drive.google.com/file/d/1u_HrG6zydMFLi4N6jeNkmE1jlnZOX5m7/view).