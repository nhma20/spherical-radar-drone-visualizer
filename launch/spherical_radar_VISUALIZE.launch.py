from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('spherical-radar-drone-visualizer'),
        'config',
        'params.yaml'
    )

    look_ahead_cone_republisher = Node(
        package="spherical-radar-drone-visualizer",
        executable="look_ahead_cone_republisher",
    )

    radar_zone_visualizer = Node(
        package="spherical-radar-drone-visualizer",
        executable="radar_zone_visualizer",
    )

    # loads robot description from URDF
    use_sim_time = LaunchConfiguration('use_sime_time', default='false')
    urdf_file_name = 'urdf/test.urdf'
    urdf = os.path.join(get_package_share_directory('spherical-radar-drone-visualizer'), urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf]
    )

    # for starting RVIZ2 in correct configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(get_package_share_directory('spherical-radar-drone-visualizer'), 'rviz/spherical_radar_VISUALIZER.rviz')]
    )


    return LaunchDescription([
        # robot_state_publisher,
        look_ahead_cone_republisher,
        radar_zone_visualizer,
        rviz_node
    ])
