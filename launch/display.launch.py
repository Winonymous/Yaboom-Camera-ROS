from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = "yahboom_camera_controller"

    use_joint_state_publisher = LaunchConfiguration('joint_state_publisher')

    use_joint_state_publisher_arg = DeclareLaunchArgument("joint_state_publisher", default_value='true')

    # Path to the rsp.launch.py
    rsp_launch_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        'rsp.launch.py'
    )

    # Path to the RViz config
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'display.rviz'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_publisher),

    )
    

    return LaunchDescription([
        use_joint_state_publisher_arg, 
        # Include rsp.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch_path)
        ),

        # Launch RViz2 with specific config
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),

        joint_state_publisher_gui
    ])
