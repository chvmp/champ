
# champ
ROS Packages for CHAMP Quadruped Controller


## 1. Installation

1.1. Clone and install all dependencies:

    sudo apt install -y python-rosdep
    cd <your_ws>/src
    git clone https://github.com/chvmp/champ
    git clone https://github.com/chvmp/champ
    cd ..
   rosdep install --from-paths src --ignore-src -r -y

1.2.  Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws/>/devel/setup.bash

## 2. Quick Start

### 2.1. Walking demo in RVIZ:

2.1.1. Run the base driver:

    roslaunch champ_config bringup.launch rviz:=true has_imu:=false

2.1.2. Run the teleop node:

    roslaunch champ_teleop teleop.launch

If you want to use a [joystick](https://www.logitechg.com/en-hk/products/gamepads/f710-wireless-gamepad.html) add joy:=true as an argument.


### 2.2. SLAM demo:

2.2.1. Run the gazebo environment:

    roslaunch champ_gazebo gazebo.launch 

2.2.2. Run gmapping package and move_base:

    roslaunch champ_config slam.launch rviz:=true

To start mapping:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- Save the map by running:

      roscd champ_config/maps
      rosrun map_server map_saver

### 2.3. Autonomous Navigation:

2.3.1. Run the gazebo environment: 

    roslaunch champ_gazebo gazebo.launch 

2.3.2. Run amcl and move_base:

    roslaunch champ_config navigate.launch rviz:=true

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)

## 3 Running your own robot:

### 3.1. Generate roobot configuration

   - First generate a configuration package using [champ_setup_assistant](https://github.com/chvmp/champ_setup_assistant). Follow the instructions in the README to configure your own robot.

   - Next, build your workspace so your newly generated package can be found:

         cd <your_ws>
         catkin_make

### 3.2.Base Driver:

The base driver contains the quadruped controller and all sensor/hardware drivers for your robot:

    roslaunch <myrobot_config> bringup.launch

Available Parameters:

rviz - Launch together with RVIZ. Default: false

has_imu - Set this to true if the robot has no IMU. This is useful when you want to view your newly configured robot. Basically, this tells the robot to use the pose commands from [champ_teleop](https://github.com/chvmp/champ_teleop) as the current pose of the robot. Take note that this is only useful for debugging the robot. It is recommended to place an IMU on a physical robot. Default: true.

lite - Set this to true if you're using a micro-controller to run the algorithms. Default false.

Example Usage:

View your newly configured robot:

    roslaunch <myrobot_config> bringup.launch rviz:true has_imu:=false

Run real robot with a micro-controller:

    roslaunch <myrobot_config> bringup.launch lite:=true


### 3.3. Creating a map:
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

### 2.3. Autonomous Navigation:

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
