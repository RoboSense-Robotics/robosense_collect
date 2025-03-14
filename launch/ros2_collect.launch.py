import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('ros2_collect').find('ros2_collect')
    collect_file_path = os.path.join(pkg_share, 'DEFAULT_CONFIG/conf/collect.yaml')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_collect', executable='ros2_collect', name='ros2_collect', output='screen', parameters=[{'collect_config_path': collect_file_path}])
    ])
