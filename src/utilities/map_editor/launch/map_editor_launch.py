import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node

def get_repo_root():
    # Search upward for the unique project configuration file
    path = os.path.dirname(os.path.abspath(__file__))
    while path != os.path.dirname(path):
        if os.path.exists(os.path.join(path, 'configs', 'f110th.config')):
            return path
        path = os.path.dirname(path)
    return None

def get_launch_source(package_name, base_name):
    try:
        pkg_share = get_package_share_directory(package_name)
    except Exception:
        return None
    py_path = os.path.join(pkg_share, 'launch', f"{base_name}.py")
    xml_path = os.path.join(pkg_share, 'launch', f"{base_name}.xml")
    if os.path.exists(py_path):
        return AnyLaunchDescriptionSource(py_path)
    elif os.path.exists(xml_path):
        return AnyLaunchDescriptionSource(xml_path)
    return AnyLaunchDescriptionSource(py_path)

def generate_launch_description():
    pkg_bringup = get_package_share_directory('race_bringup')
    pkg_map_editor = get_package_share_directory('map_editor')

    # Arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='latest',
        description="Name of the map to be used, found in assets/maps"
    )
    reverse_arg = DeclareLaunchArgument(
        'reverse',
        default_value='False',
        description='Set True if you want reverse direction of raceline'
    )
    map_editor_mapping_arg = DeclareLaunchArgument(
        'map_editor_mapping',
        default_value='False',
        description='Set True if you want to map live on the car, False to post-process.'
    )
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value=EnvironmentVariable('RACECAR_VERSION', default_value='NUC1'),
        description='Physical description of the car.'
    )

    # Configurations paths
    sensors_config = os.path.join(pkg_bringup, 'config', 'sllidar_s1.yaml')
    mux_config = os.path.join(pkg_bringup, 'config', 'mux.yaml')
    joy_teleop_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'joy_teleop.yaml'])
    vesc_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'vesc.yaml'])
    
    # Path to assets map source
    repo_root = get_repo_root()
    if repo_root is not None:
        map_src_path = PathJoinSubstitution([repo_root, 'assets/maps', LaunchConfiguration('map_name')])
    else:
        map_src_path = PathJoinSubstitution([pkg_bringup, 'maps', LaunchConfiguration('map_name')])

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Set True to launch RViz2 alongside the editor'
    )

    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_map_editor, 'rviz', 'editor.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Global Parameters Node
    global_params_node = Node(
        package='race_bringup',
        executable='global_parameter_node',
        name='global_parameters',
        parameters=[{
            'racecar_version': LaunchConfiguration('racecar_version'),
            'sim': False,
            'map_path': '',
            'use_sim_time': False
        }]
    )

    # 1. Mapping Active Group (map_editor_mapping is True)
    mapping_actions = []
    
    # Include bringup launch
    bringup_source = get_launch_source('f1tenth_stack', 'bringup_launch')
    if bringup_source is not None:
        mapping_actions.append(
            IncludeLaunchDescription(
                bringup_source,
                launch_arguments={
                    'joy_teleop_config': joy_teleop_config,
                    'vesc_config': vesc_config,
                    'sensors_config': sensors_config,
                    'mux_config': mux_config
                }.items()
            )
        )

    # Include state estimation launch
    state_estimation_source = get_launch_source('state_estimation', 'state_estimation_launch')
    if state_estimation_source is not None:
        mapping_actions.append(
            IncludeLaunchDescription(
                state_estimation_source,
                launch_arguments={
                    'racecar_version': LaunchConfiguration('racecar_version'),
                    'mapping_bool': 'True',
                    'map_name': LaunchConfiguration('map_name'),
                    'frenet_bool': 'False'
                }.items()
            )
        )

    # Global Planner (Mapping Mode)
    mapping_actions.append(
        Node(
            package='global_planner',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'global_planner', 'global_planner_params.yaml'),
                {'map_editor': True},
                {'create_map': True},
                {'map_name': LaunchConfiguration('map_name')}
            ]
        )
    )

    mapping_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('map_editor_mapping')),
        actions=mapping_actions
    )

    # 2. Offline Editing Group (map_editor_mapping is False)
    editing_actions = []
    
    # Map Server Node
    editing_actions.append(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': [map_src_path, '/', LaunchConfiguration('map_name'), '.yaml']}]
        )
    )

    # Lifecycle Manager
    editing_actions.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        )
    )

    # Global Planner (Editor Mode)
    editing_actions.append(
        Node(
            package='global_planner',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'global_planner', 'global_planner_params.yaml'),
                {'map_editor': True},
                {'create_map': False},
                {'map_name': LaunchConfiguration('map_name')},
                {'reverse_mapping': LaunchConfiguration('reverse')}
            ]
        )
    )

    # Sector Slicers
    editing_actions.append(
        Node(
            package='sector_tuner',
            executable='sector_slicer',
            name='sector_slicer',
            output='screen',
            parameters=[{'map_name': LaunchConfiguration('map_name')}]
        )
    )
    editing_actions.append(
        Node(
            package='sector_tuner',
            executable='ot_sector_slicer',
            name='ot_sector_slicer',
            output='screen',
            parameters=[{'map_name': LaunchConfiguration('map_name')}]
        )
    )

    # Global Trajectory Publisher
    global_trajectory_pub_source = get_launch_source('global_planner', 'global_trajectory_publisher_launch')
    if global_trajectory_pub_source is not None:
        editing_actions.append(
            IncludeLaunchDescription(
                global_trajectory_pub_source,
                launch_arguments={
                    'map_path': map_src_path
                }.items()
            )
        )

    editing_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('map_editor_mapping')),
        actions=editing_actions
    )

    return LaunchDescription([
        map_name_arg,
        reverse_arg,
        map_editor_mapping_arg,
        racecar_version_arg,
        rviz_arg,
        rviz_node,
        global_params_node,
        mapping_group,
        editing_group
    ])
