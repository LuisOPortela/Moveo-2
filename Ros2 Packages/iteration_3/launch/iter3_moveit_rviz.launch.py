from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import *




def generate_moveit_rviz_launch(moveit_config):
    """Launch file for rviz"""
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )	
    
    param_sim_time={
    	"use_sim_time": True
    }
    
    
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        param_sim_time,
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveo", package_name="iteration_3").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
