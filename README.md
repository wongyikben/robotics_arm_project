https://github.com/ros-controls/ros2_control_demos/tree/foxy

# Setup
Users can pull the docker image by using
```
docker pull ghcr.io/wongyikben/robotics_arm_project:latest
```
Or build the docker image after cloning the repo.
```
git clone https://github.com/wongyikben/robotics_arm_project.git
cd robotics_arm_project
docker build . -t ghcr.io/wongyikben/robotics_arm_project:latest

```

Run the following commmand to start the docker container.
```
docker run -dt --rm  --name test -p 6080:80 ghcr.io/wongyikben/robotics_arm_project:latest

```

Open the VNC by using the following url.

Click the bottom left corner and select system tool to launch the terminal.


# Packages structure
- `cr_bring`
    - contain all of the high level launch file
    - configuration file for Moveit
- `cr_description`
    - contain URDF, mesh, and RViz configuration
- `cr_planning`
    - contain MoveIt service/action client
    - contain component to filter and transform the point cloud
- `cr_simulation`
    - contain all of the information related to Gazebo simulation

## Simple Control
```
ros2 launch cr_bringup joint_state_gui_launch.py
```
This launch file is designed to verify the URDF of the robot in RViz. Users can freely move the lever to change the joint states.

## Simple Moveit2 GUI Example
```
ros2 launch cr_bringup moveit_launch.py
```
This launch file is designed to verify the moveit and ros2 control configuration using the RViz GUI. Users can move the target robot state by moving the interactive marker and press plan and execute to see how moveit reach the target position.

## Moveit2 Client Demo Example
```
ros2 launch cr_bring moveit_demo_launch.py
```
This launch file is designed to use the publisher, service client and action client to setup the obstacle, compute the FK and IK, collsion checking and plan and execute the trajectory.

The parameters for obstacle is defined in `cr_planning/config/obstacle.yaml`
The parameters for FK, IK, collsion checking and target position are defined in `cr_planning/config/planning.yaml`

The output of the FK, IK and collsion checking is printed on the terminal.

## Motion Planning with Depth Camera
```
ros2 launch cr_bringup moveit_gazebo_launch.py
```
This launch file will launch the Gazebo simulation envrionment, RViz and also `cr_planning` components, the robot arms should move around the red obstacles.
