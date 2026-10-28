from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    lu_table_arg = DeclareLaunchArgument(
        'LU_table',
        default_value='NUC5_hangar_pacejka',
        description='LU table parameter'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='MAP',
        description='Mode of operation: MAP / PP / (FTG)'
    )
    
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value='ORIN1',
        description='Hardware profile directory under race_bringup/config/ (e.g. ORIN1, SIM)'
    )
    
    controller_node = Node(
        package='controller',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[
            {'mode': LaunchConfiguration('mode')},
            {'LU_table': LaunchConfiguration('LU_table')},
            {'racecar_version': LaunchConfiguration('racecar_version')}
        ]
    )
    
    return LaunchDescription([
        lu_table_arg,
        mode_arg,
        racecar_version_arg,
        controller_node
    ])
