# Task-4-Build-Your-Own-Mobile-Robot
(ROS2 Integrated)

## 📁 File Structure
```markdown
📁 /DesignFiles
│   ├── Project Robotic Concept Design.pdf   # Complete mechanical design (exported from SolidWorks 2024)
│   ├── TB3_FullAssembly.step                # STEP file for 3D viewing/editing in CAD software
│   └── Reflection.pdf                       # Reflection on the design and integration challenges
└── README.md  
```
------
### 🤖 Overview

This repository contains the CAD design and documentation for a custom-built ROS2-integrated mobile robot platform, modeled in SolidWorks 2024. The robot is designed for:

```markdown
~ Autonomous navigation
~ Mapping (SLAM)
~ Object detection
```

It features a modular multi-tier architecture to accommodate sensors, computing units, motor drivers, and power systems—ideal for research, prototyping, and education.

### 🔧 Features

```markdown
~ 🧱 Modular Frame – Stackable plate design for easy component organization and expansion
~ 🧠 Compute Ready – Supports SBCs (e.g., Raspberry Pi, Jetson Nano) or microcontrollers
~ 🔋 Power Management – Space for onboard battery and power distribution
```

### 🎯 Sensor Suite:

```markdown
~ 🔲 2D LiDAR – For SLAM and obstacle detection
~ 🎥 Camera Module – Front-mounted for vision-based tasks
```

### ⚙️ Drive System:

```markdown
~ ⚙️ Differential Drive – Using geared DC motors
~ 🛞 Caster Wheel – For passive balancing
```

### 🛠️ Tools Used

```markdown
~ SolidWorks 2024 – For mechanical CAD modeling
~ Fusion 360 / Cura (optional) – For 3D printing components
~ PlatformIO / Arduino IDE – For firmware development
~ ROS2 – For real-time robot control and navigation
```

### 📷 Components Breakdown

```markdown
~ 🔲 LiDAR Sensor – Mounted on the top tier for 360° scanning and SLAM.
~ 🎥 Camera Module – Positioned front-facing for vision-based tasks such as object detection.
~ 🧠 PCB / Controller – Installed in the middle layers; handles computing and I/O operations.
~ ⚙️ DC Motors – Attached to the rear wheels for differential drive control.
~ 🛞 Caster Wheel – Located at the front for passive balancing and support.
~ 🔌 Power Module – Houses the battery and manages power distribution to all components.
```


### 🦾 My ROS 2 Robot Simulation with LIDAR

This project simulates a differential-drive robot with left and right wheels, a LIDAR sensor, and basic control using ros2_control. The robot is visualized in Gazebo and RViz2, and it publishes /scan data from a simulated laser.

### 📁 Project Structure

```markdown
my_robot_description/
├── config/
│   ├── my_robot_controllers.yaml
│   └── my_robot.rviz
├── launch/
│   └── my_robot_sim.launch.py
└── urdf/
    └── my_robot.urdf.xacro
```
------

### ✅ Prerequisites

```markdown
~ ROS 2 Humble (or compatible)
~ Gazebo (already included with ROS 2 desktop full)
```

Required packages:

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster
```

### 🚀 Step-by-Step Instructions
1. Setup the Project

```bash
cd ~/ros2_ws/src
```

2. Add the URDF

File: urdf/my_robot.urdf.xacro

This includes:
```markdown
~ Base box
~ Left and right wheels (with joints)
~ A LIDAR mounted on top
~ ros2_control and transmission definitions
```

```bash
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="my_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 1.0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.02"/>
      </geometry>
      <origin xyz="0 0.18 0.03" rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.02"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.02"/>
      </geometry>
      <origin xyz="0 -0.18 0.03" rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.02"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.18 0.03" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.18 0.03" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- LIDAR joint -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- LIDAR link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Gazebo sensor plugin for LIDAR -->
  <gazebo reference="lidar_link">
    <sensor name="laser" type="gpu_ray">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>30</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>640</samples>
            <resolution>1</resolution>
            <min_angle>-1.5708</min_angle>  <!-- -90 degrees -->
            <max_angle>1.5708</max_angle>   <!-- +90 degrees -->
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>3.5</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_gpu_laser.so">
        <topicName>/scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ros2_control, transmissions, etc. (unchanged) -->

</robot>
```

3. Add Controller Config

File: config/my_robot_controllers.yaml

```bash
controller_manager:
  ros__parameters:
    update_rate: 50

    diff_cont:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_cont:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.36
    wheel_radius: 0.06
    publish_rate: 50.0

    base_frame_id: base_link
    odom_frame_id: odom
    enable_odom_tf: true
    open_loop: false
    use_stamped_vel: false
    cmd_vel_timeout: 0.5
```

4. Add Launch File

File: launch/my_robot_sim.launch.py

This launch file:
```markdown
~ Starts Gazebo
~ Spawns the robot
~ Starts controller manager
~ Activates the controllers
~ Launches RViz2
```

```bash
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = '/tmp/my_robot.urdf'
    controllers_yaml = os.path.expanduser('~/ros2_ws/src/my_robot_description/config/my_robot_controllers.yaml')
    rviz_config = os.path.expanduser('~/ros2_ws/src/my_robot_description/config/my_robot.rviz')

    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf_file],
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),

        # Controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'use_sim_time': True}, urdf_file, controllers_yaml],
            output='screen'
        ),

        # Load Joint State Broadcaster
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        ),

        # Load Diff Drive Controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])
```

5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

6. Generate URDF from Xacro

```bash
ros2 run xacro xacro ~/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro -o /tmp/my_robot.urdf
```

7. Launch the Simulation

```bash
ros2 launch my_robot_description my_robot_sim.launch.py
```

🧪 Verify the LIDAR in RViz2

Once RViz2 launches:
```markdown
~ Add display: LaserScan
~ Topic: /scan
~ Frame: lidar_link
```

