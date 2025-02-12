import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    big_boy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('iteration_3'),'launch','iter3_bigboy.py'
            )])
        )
    launch_with_namespace = GroupAction(
    actions=[
        PushRosNamespace('Namespaceuwu'),
        big_boy,
    ])

    return LaunchDescription([
       launch_with_namespace

   ])

