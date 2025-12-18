import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的共享目录
    grid_path_searcher_dir = get_package_share_directory('grid_path_searcher')
    
    # RViz 配置文件路径
    rviz_config_dir = os.path.join(
        grid_path_searcher_dir,
        'launch',
        'demo.rviz'
    )

    # 声明启动参数
    map_size_x = LaunchConfiguration('map_sizze_x', default='10.0')
    map_size_y = LaunchConfiguration('map_size_y', default='10.0')
    map_size_z = LaunchConfiguration('map_size_z', default='5.0')
    start_x = LaunchConfiguration('start_x', default='0.0')
    start_y = LaunchConfiguration('start_y', default='0.0')
    start_z = LaunchConfiguration('start_z', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('map_size_x', default_value='10.0'),
        DeclareLaunchArgument('map_size_y', default_value='10.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_z', default_value='1.0'),

        # demo_node
        Node(
            package='grid_path_searcher',
            executable='demo_node',
            name='demo_node',
            output='screen',
            parameters=[{
                'map.cloud_margin': 0.0,
                'map.resolution': 0.2,
                'map.x_size': map_size_x,
                'map.y_size': map_size_y,
                'map.z_size': map_size_z,
                'planning.start_x': start_x,
                'planning.start_y': start_y,
                'planning.start_z': start_z,
            }],
            remappings=[
                ('global_map', '/random_complex/global_map')
            ]
        ),

        # random_complex_generator
        Node(
            package='grid_path_searcher',
            executable='random_complex_generator',
            name='random_complex',
            output='screen',
            parameters=[{
                'init_state_x': start_x,
                'init_state_y': start_y,
                'map.x_size': map_size_x,
                'map.y_size': map_size_y,
                'map.z_size': map_size_z,
                'map.circle_num': 40,
                'map.obs_num': 300,
                'map.resolution': 0.1,
                'map.generate_ground': 1,
                'map.max_ground_slope': 0.3,
                'ObstacleShape.lower_rad': 0.1,
                'ObstacleShape.upper_rad': 0.7,
                'ObstacleShape.lower_hei': 1.0,
                'ObstacleShape.upper_hei': 3.0,
                'CircleShape.lower_circle_rad': 0.6,
                'CircleShape.upper_circle_rad': 2.0,
                'sensing.rate': 0.5,
            }],
            remappings=[
                ('global_map', '/random_complex/global_map')
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
