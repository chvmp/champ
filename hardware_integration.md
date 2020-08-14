This contains hardware integration notes on how to connect physical robots to CHAMP controller.

Here are a few open source quadrupedal robot projects you can use to configure CHAMP with:

- [Open Quadruped](https://github.com/adham-elarabawy/OpenQuadruped)
- [openDogV2](https://github.com/XRobots/openDogV2)

## Generate Configuration Package

First, generate a configuration package using [champ_setup_assistant](https://github.com/chvmp/champ_setup_assistant). Follow the instructions in the README file to configure your own robot. The generated package contains:

- URDF path to your robot.
- Joints and Links map to help the controller know the semantics of the robot.
- Gait parameters.
- Hardware Drivers.
- Navigation parameters (move_base, amcl and gmapping).

## Actuator Controller

### Hardware Interface
In order to convert CHAMP's output (12 DOF actuator joint angles) to physical movements, you'll need to create a hardware interface for your actuators that is able to do the following:

 - Subscribe to [trajectory_msgs/JointTrajectory](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html) . This ROS message contains the joint angles (12DOF) that the actuators can use to move the robot.

- Publish all the actuators' current angle using [sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) to 'joint_states' topic.

- Control the actuators and read its angle (optional) programmatically.

One way to do this is to use [ros_control](http://wiki.ros.org/ros_control) infrastructure where you can write the necessary hardware API calls to rotate and read the current angle of the actuators. For open-loop systems like digital servo based robots, you can just store the required angles and use those as pseudo actuator feedback.

A simpler approach, is to write a ROS node that contains a publisher and subscriber to the the topics mentioned above and directly call the hardware API to rotate and read the actuator angles.

### Configuring the controller

Once you have a hardware interface, you can now include its launch file in your configuration package's bringup.launch:

    cd <your configuration package folder>/launch
    nano bringup.launch

Insert your hardware interface's launch file between the group tags. You can click [here](https://github.com/chvmp/robots/blob/master/configs/open_quadruped_config/launch/bringup.launch#L43) as an example.

You can change CHAMP's joint control topic name to match your hardware interface's trajectory_msgs/JointTrajectory msg by editing bringup.launch and changing ['joint_controller_topic'](https://github.com/chvmp/robots/blob/master/configs/open_quadruped_config/launch/bringup.launch#L15) arg to the correct topic.


Now you can launch CHAMP and your hardware driver by running:

    <my_robot_config> bringup.launch rviz:=true hardware_connected:=true

You can permanently enable 'hardware_connected' arg by setting it to 'true' inside the [bringup.launch](https://github.com/chvmp/robots/blob/master/configs/open_quadruped_config/launch/bringup.launch#L16) file.

## Mounting the actuators

All actuators when in zero position should stretch the robot's legs towards the ground. From the frontal or sagittal view, all legs should be perpendicular to the ground.

## Inertial Measurement Unit Sensor (IMU)

If you want to run the robot autonomously, you need to have an IMU installed for odometry data. You can use any IMU as long as the data is wrapped in [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) and published in 'imu/data' topic.

## Light Detection and Ranging Sensor (LIDAR)

You can use the following pre-configured LIDARs in your application:

- XV11 Lidar 
- RPLidar
- YDLIDAR X4
- Hokuyo (SCIP 2.2 Compliant)

Enable any of these supported LIDARs by using the ['laser'](https://github.com/chvmp/robots/blob/master/configs/open_quadruped_config/launch/bringup.launch#L14) arg in your configuration package's bringup.launch file:

    cd <your configuration package folder>/launch
    nano bringup.launch

change sim to any of the following:

- hokuyo
- rplidar
- xv11
- ydlidar

In the same bringup.launch file, you can define the sensor's [transform](https://github.com/chvmp/robots/blob/master/configs/open_quadruped_config/launch/bringup.launch#L39) using static_transform_publisher node.

## Foot Sensors

Foot sensors are not required for CHAMP's stock quadruped controller to work. If you're adding foot sensors for your application, the data must be wrapped in [champ_msgs/Contacts](https://github.com/chvmp/champ/blob/master/champ_msgs/msg/Contacts.msg) and published in 'foot_contacts' topic. 

The 'publish_foot_contacts' arg must also be set to false to disable the open-loop implementation in the controller:

    cd <your configuration package folder>/launch 
    nano bringup.launch

set ['publish_foot_contacts'](https://github.com/chvmp/robots/blob/master/configs/open_quadruped_config/launch/bringup.launch#L12) arg value to 'false'.
