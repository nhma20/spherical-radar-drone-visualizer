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

    tf_drone_to_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.10", "0.0", "0.0", "-1.570796", "-1.570796", "0.0", "drone", "front_frame"] #  (x, y, z, yaw, roll, pitch) # arguments=["0", "0.0", "-0.08", "1.570796", "0.0", "-1.570796", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_rear = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.10", "0.0", "0.0", "1.570796", "0.0", "0.0", "drone", "rear_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_top = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.10", "0.0", "0.10", "-1.570796", "0.0", "1.570796", "drone", "top_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_bot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0.0", "-0.10", "-1.570796", "0.0", "-1.570796", "drone", "bot_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "-0.10", "-0.04", "0.26179938", "0.0", "3.141593", "drone", "right_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.10", "-0.04", "-0.26179938", "0.0", "0.0", "drone", "left_frame"] #  (x, y, z, yaw, roll, pitch)
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
        tf_drone_to_rear,
        tf_drone_to_front,
        tf_drone_to_left,
        tf_drone_to_right,
        tf_drone_to_top,
        tf_drone_to_bot,
        # robot_state_publisher,
        look_ahead_cone_republisher,
        radar_zone_visualizer,
        rviz_node
    ])
