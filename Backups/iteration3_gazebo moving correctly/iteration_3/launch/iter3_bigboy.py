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
    
    package_name='iteration_3'

   #####################################################################
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','iter3_rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
   ####################################################################
    
    
    moveit_config = MoveItConfigsBuilder("moveo", package_name="iteration_3").to_moveit_configs()
    
    launch_package_path = moveit_config.package_path
    
    move_group = IncludeLaunchDescription(
    			PythonLaunchDescriptionSource(
    				str(launch_package_path/"launch/move_group.launch.py")
    			),            
    )
    ##################################################################
   
    rviz = IncludeLaunchDescription(
    			PythonLaunchDescriptionSource(
    				str(launch_package_path/"launch/moveit_rviz.launch.py")
    			),                
    )
    
    ##################################################################    
    
   # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
        )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_botty'],
                        output='screen')

    arm_planning_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_planning_group_controller"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )




    return LaunchDescription([
    	rsp,
        move_group,
        rviz,
        gazebo,
        spawn_entity,
        arm_planning_group_controller_spawner,
        joint_state_broadcaster_spawner
        
    ])
