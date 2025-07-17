import os

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    moveit_config = MoveItConfigsBuilder("moveo", package_name="iteration_4").to_moveit_configs()

    ld =LaunchDescription()
    
    ld.add_action (
	    Node(
	    	package="controller_manager",
	    	executable="ros2_control_node",
	    	parameters=[
	    		str(moveit_config.package_path/"config/ros2_controllers.yaml"),
	    	],
	    	remappings=[
	    		("/controller_manager/robot_description", "/robot_description"),
	    	],
	    )
    )
    ld.add_action(
    	IncludeLaunchDescription(
    		PythonLaunchDescriptionSource(
    			str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
    		),
    	)
    )
    
    
    return ld
    
