from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Paths
    urdf_file = '/tmp/my_robot.urdf'
    controllers_yaml = os.path.expanduser('~/ros2_ws/src/my_robot_description/config/my_robot_controllers.yaml')
    rviz_config = os.path.expanduser('~/ros2_ws/src/my_robot_description/config/my_robot.rviz')

    return LaunchDescription([
        # Launch Gazebo
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

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),

        # ros2_control Node (Controller Manager)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'use_sim_time': True}, urdf_file, controllers_yaml],
            output='screen'
        ),

        # Load Joint State Broadcaster
        ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'active',
                'joint_state_broadcaster'
            ],
            output='screen'
        ),

        # Load Differential Drive Controller
        ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'active',
                'diff_cont'
            ],
            output='screen'
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])

