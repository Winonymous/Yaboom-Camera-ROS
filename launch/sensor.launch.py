import os
import sys
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf_file')
    
    pkg_path = os.path.join(get_package_share_directory('yahboom_camera_controller'))
    rviz_path = os.path.join(pkg_path, "rviz", 'display.rviz')
    xacro_file = os.path.join(pkg_path,'urdf', 'servo.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config}
    

    return LaunchDescription([
        # Declare the URDF file location
        DeclareLaunchArgument('urdf_file', default_value='servo.urdf.xacro'),

        
        # Launch the robot in RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],  # Optional: use a custom RViz config file
        ),
        
        # Publish the robot's URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}],
        ),

        # Launch the joint_state_publisher_gui for controlling the joint
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    ])
