from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package path dynamically
    package_dir = os.getenv('AMENT_PREFIX_PATH').split(':')[0]
    model_path = os.path.join(package_dir, 'share', 'creating_model', 'models')
    config_path = os.path.join(package_dir, 'share', 'creating_model', 'config', 'ros_bridge.yaml')
    creating_model = get_package_share_directory('creating_model')
    

    return LaunchDescription([
        # Set Gazebo model path and launch the world
        ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                '--render-engine', 'ogre2',  # Optional: use Ogre2 rendering
                os.path.join(model_path, 'elderly_maze.sdf')
            ],
            output='screen',
            name='gazebo'
        ),
        # Node(
        #     package='ros_gz_bridge',  # Corrected package name as per deprecation notice
        #     executable='parameter_bridge',
        #     arguments=[
        #         '/camera@sensor_msgs/msg/Image[ignition.msgs.Image]',
        #         '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo]',
        #         '/imu@sensor_msgs/msg/Imu[ignition.msgs.Imu]',
        #         '/lidar/points@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan]',
        #         '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry]',
        #         '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V]',
        #         '/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist]',
        #         '/stats@ignition_msgs/msg/WorldStatistics[ignition.msgs.WorldStatistics]',
        #         '/world/elderly_maze/state@ignition_msgs/msg/WorldState[ignition.msgs.WorldState]'
        #     ],
        #     output='screen'
        # )
        Node(
            package='ros_ign_bridge',  # Or ros_gz_bridge
            executable='parameter_bridge',
            parameters=[os.path.join(creating_model, 'config', 'ros_bridge.yaml')],
            output='screen')
    ])
