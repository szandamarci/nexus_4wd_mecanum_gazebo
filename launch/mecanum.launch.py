import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'nexus_4wd_mecanum_gazebo'
    bringup_dir = get_package_share_directory('nav2_bringup')
    local_dir = get_package_share_directory(package_name)

    description_dir = get_package_share_directory('nexus_4wd_mecanum_description')
    world_file = os.path.join(description_dir, 'worlds/plain.world')
    log_world_path = LogInfo(msg=['World file path: ', world_file])
    rviz_dir = os.path.join(description_dir, 'rviz')
    xacro_file = os.path.join(description_dir, 'urdf/nexus_4wd_mecanum.xacro')

    description_raw = xacro.process_file(xacro_file).toxml()

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')


    static_tf = Node(
        package = 'tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output = 'screen'
    )

    static_tf_map = Node(
        package = 'tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
        output = 'screen'
    )

    spawn_entity = Node(package= 'gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=[
                            '-topic', 'robot_description',
                            '-entity', 'my_bot',
                            '-x', '0.0', '-y', '0.0', '-z', '0.1'
                            ],
                        output='screen')

    robot_state_publisher = Node(package= 'robot_state_publisher',
                        executable='robot_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             'use_sim_time' : use_sim_time}],
                        output='screen')
    
    joint_state_publisher = Node(package= 'joint_state_publisher',
                        executable='joint_state_publisher',
                        parameters=[
                            {'robot_description' : description_raw,
                             'use_sim_time' : use_sim_time}],
                        output='screen')
    

    rviz = Node(package= 'rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d', [os.path.join(rviz_dir, 'fourwheel.rviz')]],
                output='screen')
    
    start_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(local_dir, 'launch/nav2.launch.py')),
        launch_arguments={
            'map' : map_file,
            'use_sim_time' : 'True',
            'params_file' : params_file,
            'slam' : slam 
        }.items())
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(
            bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Path to map file'
        )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(
            description_dir, 'params', 'nav2_params_four.yaml'),
        description='Path to nav2 parameters file'
        )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', 
        default_value='True',
        description='Run slam or not'
        )
    
    declare_gui_cmd = DeclareLaunchArgument('gui', default_value='true',
                              description='Run GUI')
    
    declare_gzserver_cmd = DeclareLaunchArgument('server', default_value='true',
                              description='Run gzserver')
    

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={
                'world' : world_file,
                'use_sim_time' : 'True',
                'verbose' : 'True',
                'extra_gazebo_args': '--verbose'
            }.items()
    )

    gui= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={
                'world' : world_file,
                'use_sim_time' : 'True',
                'verbose' : 'True',
                'extra_gazebo_args': '--verbose'
            }.items()
    )


    return LaunchDescription([
        log_world_path,
        declare_gui_cmd,
        declare_gzserver_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_slam_cmd,
        gzserver,
        gui,
        spawn_entity,
        robot_state_publisher,
        static_tf,
        static_tf_map,
        joint_state_publisher, 
        rviz,
        start_nav_cmd

    ])


