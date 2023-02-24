
# champ [![Build Status](https://travis-ci.org/chvmp/champ.svg?branch=master)](https://travis-ci.org/chvmp/champ) 
ROS Packages for CHAMP Quadruped Controller.

![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

CHAMP is an open source development framework for building new quadrupedal robots and developing new control algorithms. The control framework is based on [*"Hierarchical controller for highly dynamic locomotion utilizing pattern modulation and impedance control : implementation on the MIT Cheetah robot"*](https://dspace.mit.edu/handle/1721.1/85490).

Core Features:

- Fully Autonomous (using ROS navigation Stack).
- [Setup-assistant](https://github.com/chvmp/champ_setup_assistant) to configure newly built robots.
- Collection of pre-configured [URDFs](https://github.com/chvmp/robots) like Anymal, MIT Mini Cheetah, Boston Dynamic's Spot and LittleDog. 
- Gazebo simulation environment.
- Compatible with DIY quadruped projects like [SpotMicroAI](https://spotmicroai.readthedocs.io/en/latest/) and [OpenQuadruped](https://github.com/adham-elarabawy/open-quadruped).
- Demo Applications like [TOWR](https://github.com/ethz-adrl/towr) and [chicken head](https://github.com/chvmp/chicken_head) stabilization.
- Lightweight C++ header-only [library](https://github.com/chvmp/libchamp) that can run on both SBC and micro-controllers.

Tested on:

- Ubuntu 16.04 (ROS Kinetic)
- Ubuntu 18.04 (ROS Melodic)
- Ubuntu 22.04 (ROS2 Humble)

Current state of ROS2 port:

- &check; Port libchamp.
- &cross; Port velocity smoother.
- &check; Working Gazebo empty world, without velocity control, just standing still robot with effort controllers centered.
- &check; Working rviz only demo.
- &check; Working Gazebo with teleoperated robot.
- &check; Working Gazebo demo with SLAM.
- &check; Working Gazebo demo with nav2 integration.
- &cross; Code clean-up and refactoring.
- &cross; Testing with real robot.
- &cross; Setup-Assistant.
- &cross; Robots Configurations.
    - &cross; Porting of robot description packages to ROS 2.
    - &cross; Porting of robot URDF to ROS2 (add new ros2_control tag).
    - &cross; Porting of robot configurationf to ROS2.
    - &cross; Porting of robot launch Files to ROS2.


## 1. Installation

### 1.1 Clone and install all dependencies:

    sudo apt install -y python3-rosdep
    rosdep update

    cd <your_ws>/src
    git clone --recursive https://github.com/chvmp/champ -b ros2
    git clone https://github.com/chvmp/champ_teleop -b ros2
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

If you want to use any of the pre-configured robots like Anymal, Mini Cheetah, or Spot, follow the instructions [here](https://github.com/chvmp/robots).

### 1.2 Build your workspace:

    cd <your_ws>
    colcon build
    . <your_ws>/install/setup.bash

## 2. Quick Start

You don't need a physical robot to run the following demos. If you're building a physical robot, you can find out more how to configure and run a new robot in step 3.

### 2.1 Walking demo in RVIZ:

#### 2.1.1 Run the base driver:

    ros2 launch champ_config bringup.launch.py rviz:=true 

#### 2.1.2 Run the teleop node:

    ros2 launch champ_teleop teleop.launch.py 

If you want to use a [joystick](https://www.logitechg.com/en-hk/products/gamepads/f710-wireless-gamepad.html) add joy:=true as an argument.


### 2.2 Gazebo demo:

#### 2.2.1 Run the Gazebo environment:
    
    ros2 launch champ_config gazebo.launch.py 

#### 2.2.2 Run [Nav2](https://navigation.ros.org/)'s navigation and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox):

    ros2 launch champ_config slam.launch.py rviz:=true 

To start mapping:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- Save the map by running:

      cd <your_ws>/src/champ/champ_config/maps
      ros2 run nav2_map_server map_saver_cli -f new_map

After this, you can use the new_map to do pure navigation.

### 2.3 Autonomous Navigation:

#### 2.3.1 Run the Gazebo environment: 

    ros2 launch champ_config gazebo.launch.py

#### 2.3.2 Run [Nav2](https://navigation.ros.org/):

    ros2 launch champ_config navigate.launch.py rviz:=true

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)

# All below is not yet ported for ROS2

## 3. Running your own robot:

There are two ways to run CHAMP on a real robot:

Linux Machine
- Use this ROS package to calculate the joint angles and send it to a hardware interface to control your actuators. You can follow these [guidelines](https://github.com/chvmp/champ/wiki/Hardware-Integration) to create your actuators' interface.

Lightweight Version
- Run CHAMP's [lightweight version](https://github.com/chvmp/firmware) on Teensy series microcontrollers and use it to directly control your actuators. 

### 3.1 Generate robot configuration

   - First generate a configuration package using [champ_setup_assistant](https://github.com/chvmp/champ_setup_assistant). Follow the instructions in the README to configure your own robot. The generated package contains:

        - URDF path to your robot.
        - Joints and Links map to help the controller know the semantics of the robot.
        - Gait parameters.
        - Hardware Drivers.
        - Navigation parameters (move_base, amcl and gmapping).
        - Microcontroller header files for gait and lightweight robot description. This only applies to robot builds that use microcontroller to run the quadruped controller.

     As a reference, you can check out the collection of robots that have been pre-configured [here](https://github.com/chvmp/robots). In the list are some of the popular quadruped robots like Anymal, MIT Mini Cheetah, Boston Dynamic's LittleDog, and SpotMicroAI. Feel free to download the configuration packages in your catkin workspaces 'src' directory to try.

   - Next, build your workspace so your newly generated package can be found:

         cd <your_ws>
         catkin_make

### 3.2 Base Driver:

This will run the quadruped controller and all sensor/hardware drivers:

    roslaunch <myrobot_config> bringup.launch

Available Parameters:

  - **rviz** - Launch together with RVIZ. Default: false

  - **lite** - Always set this to true if you're using a microcontroller to run the algorithms. Default false.

Example Usage:

View your newly configured robot:

    roslaunch <myrobot_config> bringup.launch rviz:true
    
Run real robot with a microcontroller:

    roslaunch <myrobot_config> bringup.launch lite:=true


### 3.3 Creating a map:
The base driver described in 3.2 must be running to run gmapping and move_base.

Run gmapping package and move_base:

    roslaunch <myrobot_config> slam.launch

To open RVIZ and view the map:

    roscd champ_navigation/rviz 
    rviz -d navigate.rviz

To start mapping:
- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- Save the map by running:

      roscd <myrobot_config>/maps
      rosrun map_server map_saver

### 3.4 Autonomous Navigation:

The base driver described in 3.2 must be running to run amcl and move_base.

Run amcl and move_base:

    roslaunch <myrobot_config> navigate.launch

To open RVIZ and view the map:

    roscd champ_navigation/rviz 
    rviz -d navigate.rviz

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)

### 3.5 Running your robot in Gazebo

Run Gazebo and the base driver in simulation mode:

    roslaunch <myrobot_config> gazebo.launch

* Take note that in order for this to work, the URDF has to be Gazebo compatible and has [ros_control](http://gazebosim.org/tutorials/?tut=ros_control) capability. The controllers have been set-up so all you need is to add the transmission of the actuators. You also need to get the physics parameters right like your mass, inertia, and foot friction. 

   Some useful resources on getting these parameters right:

  - Inertial Calculation - https://github.com/tu-darmstadt-ros-pkg/hector_models/blob/indigo-devel/hector_xacro_tools/urdf/inertia_tensors.urdf.xacro

  - List of Moment of Inertia - https://en.wikipedia.org/wiki/List_of_moments_of_inertia

  - Gazebo inertial parameters - http://gazebosim.org/tutorials?tut=inertia&cat=build_robot#Overview

You can also check out [this](https://github.com/moribots/spot_mini_mini/pull/7) pull request as an example.

### 3.6 Spawning multiple robots in Gazebo

Run Gazebo and default simulation world:

    roslaunch champ_gazebo spawn_world.launch 

You can also load your own world file by passing your world's path to 'gazebo_world' argument:

    roslaunch champ_gazebo spawn_world.launch gazebo_world:=<path_to_world_file>

Spawning a robot:

    roslaunch champ_config spawn_robot.launch robot_name:=<unique_robot_name> world_init_x:=<x_position> world_init_y:=<y_position>

    
* Every instance of the spawned robot must have a unique robot name to prevent the topics and transforms from clashing.


## 4. Tuning gait parameters

The gait configuration for your robot can be found in <my_robot_config>/gait/gait.yaml.

![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/gait_parameters.png)

- **Knee Orientation** - How the knees should be bent. You can can configure the robot to follow the following orientation .>> .>< .<< .<> where dot is the front side of the robot.

- **Max Linear Velocity X** (meters/second) - Robot's maximum forward/reverse speed.

- **Max Linear Velocity Y** (meteres/second) - Robot's maximum speed when moving sideways.

- **Max Angular Velocity Z** (radians/second)- Robot's maximum rotational speed.

- **Stance Duration** (seconds)- How long should each leg spend on the ground while walking. You can set this to default(0.25) if you're not sure. The higher the stance duration the further the displacement is from the reference point.

- **Leg Swing Height** (meters)- Trajectory height during swing phase.

- **Leg Stance Height** (meters)- Trajectory depth during stance phase.

- **Robot Walking Height** (meters) - Distance from hip to the ground while walking. Take note that setting this parameter too high can get your robot unstable.

- **CoM X Translation** (meters) - You can use this parameter to move the reference point in the X axis. This is useful when you want to compensate for the weight if the center of mass is not in the middle of the robot (from front hip to rear hip). For instance, if you find that the robot is heavier at the back, you'll set a negative value to shift the reference point to the back.

- **Odometry Scaler** - You can use this parameter as a multiplier to the calculated velocities for dead reckoning. This can be useful to compensate odometry errors on open-loop systems. Normally this value ranges from 1.0 to 1.20.
